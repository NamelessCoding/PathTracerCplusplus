// FractalRenderer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

//#include "drawing.h"

#include <iostream>
#include <thread>
#include <stdio.h>
#include <cmath>
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <cmath>
#include <assert.h>
#include <stdint.h>

//#include "drawing.h"

#include <random>
#include <vector>
#include "ray.h"
#include "shapes.h"
#include "info.h"

#include <string>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "vec.h"
#define FORCE_SINGLE_THREAD() 0

extern "C" {
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
}

bool load_image(std::vector<unsigned char>& image, const std::string& filename, int& x, int& y)
{
	int n;
	unsigned char* data = stbi_load(filename.c_str(), &x, &y, &n, 4);
	if (data != nullptr)
	{
		image = std::vector<unsigned char>(data, data + x * y * 4);
	}
	stbi_image_free(data);
	return (data != nullptr);
}

float modulo(float a, float b) {
	//\left(\frac{ x }{2} - \operatorname{ floor }\left(\frac{ x }{2}\right)\right)\cdot2
	return (a / b - floorf(a/b))*b;
}

int max_depth = 5;

const size_t xres = 1280;
const size_t yres = 720;

std::string filename = "hdr.png";

int width, height;
std::vector<unsigned char> hdr;

const size_t RGBA = 4;

std::vector<uint32_t> image(xres* yres, 0);

float schlickWeight(float cosTheta) {
	float m = clamp(1.f - cosTheta, 0.f, 1.f);
	return (m * m) * (m * m) * m;
}
#define EPSILON 0.0001f
#define PI 3.14159f
float pow2(float a) {
	return a * a;
}

float GTR1(float NdotH, float a) {
	if (a >= 1.f) return 1.f / PI;
	float a2 = a * a;
	float t = 1.f + (a2 - 1.f) * NdotH * NdotH;
	return (a2 - 1.f) / (PI * log(a2) * t);
}

float GTR2(float NdotH, float a) {
	float a2 = a * a;
	float t = 1.f + (a2 - 1.f) * NdotH * NdotH;
	return a2 / (PI * t * t);
}

float GTR2_aniso(float NdotH, float HdotX, float HdotY, float ax, float ay) {
	return 1.f / (PI * ax * ay * pow2(pow2(HdotX / ax) + pow2(HdotY / ay) + NdotH * NdotH));
}

float smithG_GGX(float NdotV, float alphaG) {
	float a = alphaG * alphaG;
	float b = NdotV * NdotV;
	return 1.f / (abs(NdotV) + std::max(sqrt(a + b - a * b), EPSILON));
}

float smithG_GGX_aniso(float NdotV, float VdotX, float VdotY, float ax, float ay) {
	return 1.f / (NdotV + sqrt(pow2(VdotX * ax) + pow2(VdotY * ay) + pow2(NdotV)));
}

float mix(float a, float b, float d) {
	return a * (1.0 - d) + b * d;
}

vec3<float> mix(vec3<float> a, vec3<float> b, float d) {
	return vec3<float>(a.e[0] * (1.0 - d) + b.e[0] * d,
		a.e[1] * (1.0 - d) + b.e[1] * d, 
		a.e[2] * (1.0 - d) + b.e[2] * d);
}

vec3<float> mix(vec3<float> a, vec3<float> b, vec3<float> d) {
	return vec3<float>(a.e[0] * (1.0 - d.e[0]) + b.e[0] * d.e[0],
		a.e[1] * (1.0 - d.e[1]) + b.e[1] * d.e[1],
		a.e[2] * (1.0 - d.e[2]) + b.e[2] * d.e[2]);
}

vec3<float> disneyDiffuse(float NdotL,  float NdotV, float LdotH, float roughness, vec3<float> baseColor) {

	float FL = schlickWeight(NdotL), FV = schlickWeight(NdotV);

	float Fd90 = 0.5f + 2.f * LdotH * LdotH * roughness;
	//x×(1−a)+y×a
	float Fd = mix(1.0f, Fd90, FL) * mix(1.0f, Fd90, FV);

	return baseColor * (1.0f/3.14159f) * Fd;
}

vec3<float> disneySubsurface(float NdotL, float NdotV,
	float LdotH, float roughness, vec3<float> baseColor) {

	float FL = schlickWeight(NdotL), FV = schlickWeight(NdotV);
	float Fss90 = LdotH * LdotH * roughness;
	float Fss = mix(1.0f, Fss90, FL) * mix(1.0f, Fss90, FV);
	float ss = 1.25f * (Fss * (1.f / (NdotL + NdotV) - .5f) + .5f);

	return baseColor* ss * (1.f / PI);
}

vec3<float> disneyMicrofacetIsotropic(float NdotL, float NdotV, float NdotH, float LdotH,
	float roughness, vec3<float> baseColor, vec3<float> specular, vec3<float> specularTint, float metallic) {

	float Cdlum = .3f * baseColor.e[0] +
		.6f * baseColor.e[1] + .1f * baseColor.e[2]; // luminance approx.

	vec3<float> Ctint = Cdlum > 0.f ? baseColor / Cdlum : vec3<float>(1.f,1.f,1.f); // normalize lum. to isolate hue+sat
	vec3<float> Cspec0 = mix(specular * .08f * mix(vec3<float>(1.f,1.f,1.f), Ctint,
		specularTint), baseColor, metallic);

	float a = std::max(.001f, pow2(roughness));
	float Ds = GTR2(NdotH, a);
	float FH = schlickWeight(LdotH);
	vec3<float> Fs = mix(Cspec0, vec3<float>(1.f,1.f,1.f), FH);
	float Gs;
	Gs = smithG_GGX(NdotL, a);
	Gs *= smithG_GGX(NdotV, a);

	return Fs * Gs * Ds;
}

vec3<float> disneySheen(float LdotH, vec3<float> baseColor, vec3<float> sheenTint, vec3<float> sheen) {
	float FH = schlickWeight(LdotH);
	float Cdlum = .3f * baseColor.e[0] + .6f * baseColor.e[1] + .1f * baseColor.e[2];

	vec3<float> Ctint = Cdlum > 0.f ? baseColor / Cdlum : vec3<float>(1.f,1.f,1.f);
	vec3<float> Csheen = mix(vec3<float>(1.f,1.f,1.f), Ctint, sheenTint);
	vec3<float> Fsheen =  sheen * Csheen * FH;
	return  sheen * Csheen * FH;
}


float chiGGX(float v)
{
	return v > 0 ? 1 : 0;
}

float GGX_PartialGeometryTerm(float dh, float dn, float alpha)
{
	float VoH2 = dh;
	float chi = chiGGX(VoH2 / dn);
	VoH2 = VoH2 * VoH2;
	float tan2 = (1 - VoH2) / VoH2;
	return (chi * 2) / (1 + sqrt(1 + alpha * alpha * tan2));
}

float DistributionGGX(vec3<float> N, vec3<float> H, float roughness)
{
	float a = roughness * roughness;
	float a2 = a * a;
	float NdotH = std::max(N.dot(N, H), 0.0f);
	float NdotH2 = NdotH * NdotH;

	float num = a2;
	float denom = (NdotH2 * (a2 - 1.0f) + 1.0f);
	denom = 3.14159f * denom * denom;

	return num / denom;
}

float GeometrySchlickGGX(float NdotV, float roughness)
{
	float a = roughness;
	float k = (a * a) / 2.0;

	float num = NdotV;
	float denom = NdotV * (1.0 - k) + k;

	return num / denom;
}
float GeometrySmith(vec3<float> N, vec3<float> V, vec3<float> L, float roughness)
{
	float NdotV = std::max(N.dot(N, V), 0.0f);
	float NdotL = std::max(N.dot(N, L), 0.0f);
	float ggx2 = GeometrySchlickGGX(NdotV, roughness);
	float ggx1 = GeometrySchlickGGX(NdotL, roughness);

	return ggx1 * ggx2;
}

vec3<float> EnvBRDFApprox(vec3<float> f0, float NoV, float roughness)
{
	//float4 c0 = float4(-1.0, -0.0275, -0.572, 0.022);
	vec3<float> c0 = vec3<float>(-1.0f, -0.0275f, -0.572f);
	float c0w = 0.022;
	//float4 c1 = float4(1.0, 0.0425, 1.04, -0.04);
	vec3<float> c1 = vec3<float>(1.0f, 0.0425f, 1.04f);
	float c1w = -0.04;
	vec3<float> r = c0 * roughness + c1;
	float rw = c0w * roughness + c1w;

	//float a004 = min(r.x * r.x, exp2(-9.28 * NoV)) * r.x + r.y;
	float a004 = std::min(r.e[0]*r.e[0],exp2(-9.28f * NoV) ) * r.e[0] + r.e[1];
	//float2 AB = float2(-1.04, 1.04) * a004 + r.zw;
	float ABx = -1.04 * a004 + r.e[2];
	float ABy = 1.04 * a004 + rw;
	return f0 * ABx + ABy;
}

vec3<float> ct(float r, float m, vec3<float> direction, vec3<float> normal,
	float rnd1, float rnd2, vec3<float> origin, vec3<float> color, vec3<float> lightdir) {
	//r = 1.0 - r;
	//vec3<float>(0.f, 6.5f, 5.3f), vec3<float>(4.3f, 4.3f, 0.3f)
	//vec3<float>(-12.f, 6.5f, 0.3f), vec3<float>(0.3f, 18.3f, 18.3f)
	//vec3<float>(-12.f, 6.5f, 0.3f), vec3<float>(0.3f, 18.3f, 18.3f)
	//vec3<float>(0.f, 6.5f, 6.3f), vec3<float>(18.3f, 18.3f, 0.3f)
    vec3<float> lightpos = vec3<float>(0.f, 6.5f, 6.3f);
	

	float ar = rnd1 * 2.0f - 1.0f;
	float br = rnd2 * 2.0f - 1.0f;
	lightpos.e[0] += ar * 3.3f;
	lightpos.e[1] += br * 3.3f;
	//direction = lightdir;
	direction = -direction;
	lightdir = lightpos - origin;
	lightdir.normalize();
	vec3<float> h = direction + lightdir;
	h.normalize();
	//float nh = std::max(normal.dot(normal, h),0.f);
	//float d1 = 1.0f / (r*r*nh*nh*nh*nh);
	//float d2 = exp( (nh*nh - 1.0f)/(r*r*nh*nh));
	//float D = d1 * d2;
	float D = DistributionGGX(normal, h, r);


	vec3<float> f0 = vec3<float>(0.04f,0.04f, 0.04f);
	//x×(1−a)+y×a
	//float rr = map.r * map.r;
	//spec = spec * (1.0f - rr) + diff * rr;

	//vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness)
	//{
	//	return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(max(1.0 - cosTheta, 0.0), 5.0);
//	}

	f0 = f0 * (1.0f - m) + color * m;
	vec3<float> env = EnvBRDFApprox(f0, std::max(direction.dot(direction, normal), 0.f), r);
	float ftheta = std::max(1.0f - std::max(direction.dot(direction, normal),0.0f), 0.f);
	vec3<float> f1 = vec3<float>(1.f - r, 1.f - r, 1.f - r);
	f1 = f1.maxim(f1, f0);
	vec3<float> F = f0 + (f1-f0 ) * powf(ftheta, 5.0f);
	//F = F * env.e[0] + env.e[1];

	//float nd = std::max(direction.dot(direction, normal),0.f);
	//float dh = std::max(direction.dot(direction, h),0.f);
	//float ln = std::max(lightdir.dot(lightdir, normal),0.f);
	//float s1 = (2.0f * nh * nd) / (dh);
	//float s2 = (2.0f * nh * ln) / (dh);
	//float G = std::min(1.f, std::min(s1,s2));
	/*float GGX_PartialGeometryTerm(float dh, float dn,
		float alpha)
	{
		float VoH2 = dh;
		float chi = chiGGX(VoH2 / dn);*/

	//float G = GGX_PartialGeometryTerm(dh, nd, r) 
		  //  * GGX_PartialGeometryTerm(std::max(lightdir.dot(lightdir,h), 0.f),ln,r);
	float G = GeometrySmith(normal, direction, lightdir, r);

	vec3<float> top = F * D * G;
	float bottom = 3.14159f * std::max(normal.dot(normal, direction),0.f) * std::max(normal.dot(normal, lightdir),0.f);
	vec3<float> spec = (top / std::max(bottom, 0.001f));


	vec3<float> kS = F;
	vec3<float> kD = vec3<float>(1.0f,1.f,1.f) - kS;
	kD *= 1.0 - m;
	//float NdotL = std::max(d, 0.0);
	return (kD * color/3.14159 + spec);

}

vec3<float> SampleLight(all_shapes<float> map, ray<float> r, vec3<float> normal, float rnd1, float rnd2) {
	//vec3<float>(0.f, 0.5f, 4.5f), vec3<float>(3.3f, 3.3f, 0.3f)
    bool hit_object = false;
	//vec3<float>(0.f, 6.5f, 5.3f), vec3<float>(3.3f, 3.3f, 0.02f)
	vec3<float> lightpos = vec3<float>(-20.f, 0.5f, 0.3f);
	//vec3<float>(0.f, 5.5f, 0.5f), vec3<float>(3.3f, 0.3f, 3.3f)
	//vec3<float>(-20.f, 6.5f, 15.3f), vec3<float>(0.3f, 12.3f, 12.3f)
	float ar = rnd1*2.0f - 1.0f;
	float br = rnd2*2.0f - 1.0f;
	lightpos.e[1] += ar * 12.3f;
	lightpos.e[2] += br * 12.3f;
	//r.origin += normal*0.1f;
	r.direction = lightpos - r.origin;
	float lengthSquared = r.direction.length();
	lengthSquared = lengthSquared * lengthSquared;
	r.direction.normalize();
	float cosa = std::max(normal.dot(normal, r.direction),0.f);
	info<float> inf;
	inf = map.intersect(r.origin, r.direction);
	hit_object = inf.hit;
	//
	vec3<float> normal2 = vec3<float>(1.f, 0.f, 0.f);
	if (r.origin.e[0] < lightpos.e[0]) {
		normal2 = vec3<float>(-1.f, 0.f, 0.f);
	}
	if (hit_object && inf.l > 0.01 && normal.dot(normal, r.direction) > 0. && r.direction.dot(-r.direction, normal2) > 0.) {
		
		float area = 12.3 * 12.3;
		//DistanceSquared(ref.p, isectLight.p) /(AbsDot(isectLight.n, -wi) * Area());
		float ans = lengthSquared / (std::abs(normal2.dot(normal2, -r.direction))*area);

		//float ans = r2 / (abs(dot(ln, -ligdir)) * area);
		//float im = l * 0.76 * dot(n, ligdir);
		//color += (tt * im) / ans;
		return vec3<float>(inf.l * (std::max(normal.dot(normal, r.direction),0.f)/(ans + (std::max(r.direction.dot(normal, r.direction), 0.0f) / 3.14159f))), 1., 0.);
	}
	return vec3<float>(0.,1.,1.);
}

vec3<float> ortho(vec3<float> v) {
	//  See : http://lolengine.net/blog/2013/09/21/picking-orthogonal-vector-combing-coconuts
	return abs(v.e[0]) > abs(v.e[2]) ? vec3<float>(-v.e[1], v.e[0], 0.0) : vec3<float>(0.0, -v.e[2], v.e[1]);
}

vec3<float> hemi(float rnd1, float rnd2, float rnd3, vec3<float> normal, float s, float r, vec3<float> dir) {
	float z = rnd1 * 2.0f - 1.0f;
	float a = rnd2 * 2.0f * 3.14159f;
	float r2 = sqrt(1.0f - z * z);
	float x = r2 * cos(a);
	float y = r2 * sin(a);
	vec3<float> random_sphere = vec3<float>(x, y, z);

	/*float th = rnd1 * 3.14159f * 2.0f;
	float ph = rnd2 * 3.14159f;

	vec3<float> random_sphere = vec3<float>(sin(ph) * cos(th),sin(ph) * sin(th),cos(ph));*/

	/*vec3<float> m = vec3<float>(rnd1*2.0-1.0, rnd2*2.0-1.0,rnd3*2.0-1.0);
	m.normalize();
	if (normal.dot(normal, m) <= 0.) {
		m *= -1.;
	}

	vec3<float> x = normal.cross(normal, m);
	x.normalize();
	vec3<float> y = normal.cross(normal, x);
	y.normalize();

	float phi = rnd1 * 3.14159 * 2.0;
	float theta = acos(std::sqrt(rnd2));

	vec3<float> k = vec3<float>(
		cos(2. * 3.14159f * rnd1) * sqrt(1. - rnd2),
		sin(2. * 3.14159f * rnd1) * sqrt(1. - rnd2),
		sqrt(rnd2));

	vec3<float> n1 = x * k.e[0];
	vec3<float> n2 = y * k.e[1] ;
	vec3<float> n3 = normal * k.e[2];
	vec3<float> random_sphere = n1 + n2 + n3;
	random_sphere.normalize();
	*/
	

	//r.direction = normal + random_sphere;

	float doSpecular = (rnd3 < s) ? 1.0f : 0.0f;

	vec3<float> diff = random_sphere;
	diff.normalize();
	vec3<float> spec = dir.reflect(dir, normal);
	//specularRayDir = normalize(mix(specularRayDir, diffuseRayDir, r * r));
	//x×(1−a)+y×a
	float rr = r * r;
	spec = spec * (1.0f - rr) + diff * rr;
	spec.normalize();
	vec3<float> direc = diff * (1.0f - doSpecular) + spec * doSpecular;
	//d = mix(diffuseRayDir, specularRayDir, doSpecular);
	direc.normalize();
	return direc;
}

float clamp(float a, float b, float c) {
	return (a<b) ? b: (a>c)?c:a;
}

vec3<float> Trace(all_shapes<float> map, ray<float> r, int depth, float uvx, int x, int y) {
	std::random_device rd;
	std::mt19937_64 gen(rd());
	std::uniform_real_distribution<float> rnd(0.f, 1.f);

	vec3<float> intensity = vec3<float>(0.f,0.f,0.f);
	vec3<float> tt = vec3<float>(1.f,1.f,1.f);
	if (depth >= max_depth) {
		return intensity;
	}

	vec3<float> constv = r.direction;

	bool hit_object = false;
	bool prevrough = false;
	bool last_ref = false;
	for (int bounce = 0; bounce < 37; bounce++) {
		info<float> inf;
		inf = map.intersect(r.origin, r.direction);
		hit_object = inf.hit;

		if (hit_object) {
			/*intensity.e[0] = 0.9f;
			intensity.e[1] = 0.4f;
			intensity.e[2] = 0.4f;
			*/
			r.origin += r.direction * inf.distance;
			r.origin = r.origin - r.direction * 0.03f;
			if (r.direction.dot(r.direction, inf.normal) > 0.01) {
				inf.normal = -inf.normal;
			}
			vec3<float> normal = inf.normal;
			normal.normalize();

			/*intensity.e[0] = (inf.col.e[0]);
			intensity.e[1] = (inf.col.e[1]);
			intensity.e[2] = (inf.col.e[2]);
			
			break;*/
			
			r.origin = r.origin + normal * 0.03f;

			//const float p = 1.f / (2.f * 3.14159f);

			//EmittedLight + 2 * RecursiveLight * Dot(Normal, RandomHemisphereAngle) * SurfaceDiffuseColor.

			

		
			//float dot2 = normal.dot(normal, r.direction);
			//intensity = vec3<float>(map.l, map.l, map.l);
			//ct(float r, float m, vec3<float> direction, vec3<float> normal, float rnd1, float rnd2, vec3<float> origin)
			//vec3<float> incomming = Trace(map, r, depth + 1);
			//hemi(float rnd1, float rnd2, float rnd3, vec3<float> normal, float s, float r, vec3<float> dir) 
			//vec3<float> hem = hemi(rnd(gen), rnd(gen), rnd(gen), normal, inf.s, inf.r, r.direction);
		    //float m = (map.r < 0.98f) ? 0.7f : 0.0f;
			//vec3<float> brdf = ct(inf.r, 0.f, r.direction, normal, rnd(gen), rnd(gen), r.origin, inf.col, hem);

			
			/*if (inf.l > 0.01) {
				intensity.e[0] = 1.;
				intensity.e[1] = 1.;
				intensity.e[2] = 1.;
				break;
			}
			else {
				intensity.e[0] = brdf.e[0];
				intensity.e[1] = brdf.e[1];
				intensity.e[2] = brdf.e[2];
				break;
			}*/

			vec3<float> wo = -r.direction;
			vec3<float> dir = r.direction;
			if (inf.ref) {
				if (r.direction.dot(r.direction, inf.normal) > 0.01) {
					inf.normal = -inf.normal;
					r.direction = r.direction.refract(inf.normal, r.direction, 1.0f, 1.25f);
					r.origin = r.origin - inf.normal * 0.1f;
				}
				else {
					r.direction = r.direction.refract(inf.normal, r.direction, 1.0f, 1.5f);
					r.origin = r.origin - inf.normal * 0.1f;
				}
				//continue;
			}
			else {
				r.direction = hemi(rnd(gen), rnd(gen), rnd(gen), normal, inf.s, inf.r, r.direction);
			}
			
			//tt.e[0] *=brdf.e[0];
			//tt.e[1] *=brdf.e[1];
			//tt.e[2] *=brdf.e[2];

			//cos(d) is dot(wi, wh) where wh = normalize(wi + wo)
			vec3<float> wi = r.direction;
			//wi = vec3<float>(0.f, 6.5f, 15.3f) - r.origin;
			//wi.normalize();
			vec3<float> wh = wi + wo;
			wh.normalize();


			float cosd = clamp(wi.dot(wi, wh), 0., 1.);
			float cosl = clamp(normal.dot(normal, wi), 0., 1.);
			float cosv = clamp(normal.dot(normal, wo), 0., 1.);
			float costh = clamp(normal.dot(normal, wh), 0., 1.);
			float roughness = 1.;
			vec3<float> specular = vec3<float>(3.0f, 3.0f, 3.0f);

			//float spe = 0.0f;
			float metalic = 0.0f;
			//float sub = 0.9f;
			vec3<float> sheenTint = vec3<float>(0.93f, 0.93f, 0.93f);
			vec3<float> sheen = vec3<float>(0.99f, 0.99f, 0.99f);
			vec3<float> disn = disneyMicrofacetIsotropic(cosl, cosv, costh, cosd,
				roughness, inf.col, specular, vec3<float>(0.43f, 0.43f, 0.43f), metalic);
			vec3<float> disnsheen = disneySheen(cosd, inf.col, sheenTint, sheen);
			//vec3<float> disnsub = disneySubsurface(cosl, cosv, cosd, roughness, inf.col);
			vec3<float> disndiff = disneyDiffuse(cosl, cosv, cosd, roughness, inf.col);
			vec3<float> f = (disndiff + disnsheen) * (1.f - metalic);
			f += disn;

			info<float> inf2;
			
			vec3<float> wor = wo.reflect(dir, inf.normal);
			inf2 = map.intersect(r.origin, wor);

			if (inf2.l > 0.01 && wo.dot(wor, vec3<float>(-1.,0.,0.)) > 0.96) {
				f += 0.2;
			}


			/*intensity.e[0] = disn.e[0];
			intensity.e[1] = disn.e[1];
			intensity.e[2] = disn.e[2];
			break;*/
			//float btD = roughness*roughness*wh.e[2]*wh.e[2] + 
			//brdfEval(wo, wi) * abs(dot(wi, normal))
			
		if(true){
 		//if ((x/62+y/62) % 2 == 0) {
	    //if(uvx > 0.0f){
				if ((bounce == 0) && inf.l > 0.01) {
					intensity.e[0] += tt.e[0] * inf.l;
					intensity.e[1] += tt.e[1] * inf.l;
					intensity.e[2] += tt.e[2] * inf.l;
					//return intensity;
					continue;
				}
				else if (bounce > 0 && inf.l > 0.01) {
					return intensity;
				}


				if (inf.l < 0.01) {
					float rnd1 = rnd(gen);
					float rnd2 = rnd(gen);

					vec3<float> L = SampleLight(map, ray<float>(r.origin, r.direction, 0.f), normal, rnd1, rnd2);
					//L = std::min(std::max(L, 0.f),1.f);
					
					intensity.e[0] +=(tt.e[0] * (f.e[0]) * L.e[0])/L.e[1];
					intensity.e[1] +=(tt.e[1] * (f.e[1]) * L.e[0])/L.e[1];
					intensity.e[2] +=(tt.e[2] * (f.e[2]) * L.e[0])/L.e[1];
				}
				/*else if (map.l > 0.01) {
					intensity.e[0] += tt.e[0] * map.l;
					intensity.e[1] += tt.e[1] * map.l;
					intensity.e[2] += tt.e[2] * map.l;
				}*/
			}
			else {
				if (inf.l > 0.01) {
					//0.5 + 0.5 * cos(iTime + uv.xyx + vec3(0, 2, 4));
					
					intensity.e[0] += tt.e[0] * inf.l;
					intensity.e[1] += tt.e[1] * inf.l;
					intensity.e[2] += tt.e[2] * inf.l;
				}
			}
			
		float hemipdf = (wi.dot(wi, normal)) / (3.14159f);
		tt.e[0] *= ((f.e[0]) * std::max(wi.dot(wi, normal), 0.f)) / hemipdf;
		tt.e[1] *= ((f.e[1]) * std::max(wi.dot(wi, normal), 0.f)) / hemipdf;
		tt.e[2] *= ((f.e[2]) * std::max(wi.dot(wi, normal), 0.f)) / hemipdf;

			//intensity.e[0] *= brdf.e[0];
			//intensity.e[1] *= brdf.e[1];
			//intensity.e[2] *= brdf.e[2];
			
			if (bounce > 3) {
				float p = std::max(tt.e[0], std::max(tt.e[1], tt.e[2]));
				if (rnd(gen) > p) {
					break;
				}
				tt *= 1.f / p;
			}

			if (!last_ref && inf.r < 0.99) {
				last_ref = true;
			}
			else {
				last_ref = false;
			}

		}
		else
		{
			


			float dx = (r.direction.e[0] + 1.0f)*0.5f;
			int x = (int)((float)width * dx);
			float dz =1.0f - (r.direction.e[2] + 1.0f) * 0.5f;
			int y = (int)((float)height * dz);
			size_t index = RGBA * (y * width + x);
			//std::cout << "RGBA pixel @ (x=3, y=4): "
			float r = static_cast<float>(hdr[index + 0]) / 255.f;
			float g = static_cast<float>(hdr[index + 1]) / 255.f;
			float b = static_cast<float>(hdr[index + 2]) / 255.f;

			/*vec3<float> ld = vec3<float>(0.f,1.f,0.2f);
			ld.normalize();
			float thetas = ld.dot(ld, vec3<float>(0.f, 0.f, 1.f));
			float theta = r.direction.dot(r.direction, vec3<float>(0.f, 0.f, 1.f));
			if (theta < 0.03)return intensity;
			float thetay = r.direction.dot(r.direction, ld);
			float y = acos(thetay);
			float taa = r.direction.dot(-r.direction, vec3<float>(0.f, 0.f, 1.f));
			float tay = r.direction.dot(-r.direction, ld);
			float Fred = (1.0 - 1.32004 * exp(0.14318 / taa)) * (1.0 + 5.30694 * exp(-2.48062 * y) + 0.3167 * tay * tay);
			float F2 = 1.0 - (-0.523242) * (1.0 + 5.30694 * exp(-2.48062 * y) + 0.3167 * thetas * thetas);
			float Fblue = (1.0 + -0.27416 * exp(-0.0668 / taa)) * (1.0 + 0.20388 * exp(-1.68898 * y) + 0.04418 * tay * tay);
			//return 0.2 * vec3((1.0 - thetas) * Fred, Fblue * 0.4 * max(ld.z, 0.), Fblue)
			//	+ F2 * F2 * vec3(0.1, 0.07, (thetas) * 0.1);
			vec3<float> fin1 = vec3<float>((1.0 - thetas) * Fred, Fblue * 0.4 * std::max(ld.e[2], 0.f), Fblue) * 0.2;
			vec3<float> fin2 = vec3<float>(0.1, 0.07, (thetas) * 0.1)*F2*F2;

			vec3<float> finc = fin1 + fin2;*/
			intensity.e[0] += 0. * tt.e[0];
			intensity.e[1] += 0. * tt.e[1];
			intensity.e[2] += 0. * tt.e[2];
			return intensity;
		}
		if (map.r < 0.98) {
			prevrough = true;
		}
		else {
			prevrough = false;
		}
		//hit_object = false;
	}
	return intensity;
}

std::atomic<size_t> g_currentRowIndex(-1);

vec3<float> ACESFilm(vec3<float> x)
{
	float a = 2.51f;
	float b = 0.03f;
	float c = 2.43f;
	float d = 0.59f;
	float e = 0.14f;
	vec3<float> fin = ((x * ((x * a) + b)) / (x * ((x * c) + d) + e));
	fin.e[0] = (fin.e[0] < 0.0f) ? 0.f : (fin.e[0] > 1.0f)? 1.0f : fin.e[0];
	fin.e[1] = (fin.e[1] < 0.0f) ? 0.f : (fin.e[1] > 1.0f) ? 1.0f : fin.e[1];
	fin.e[2] = (fin.e[2] < 0.0f) ? 0.f : (fin.e[2] > 1.0f) ? 1.0f : fin.e[2];
	return fin;
}

void thread_func(all_shapes<float> map) {
	int keep_track = 0;
	const int num_samples = 100;
	size_t y = ++g_currentRowIndex;

	while (y < yres){
		int pos = (yres - 1) - y;
		uint32_t* pixel = &image[pos * xres];
		for (size_t x = 0; x < xres; ++x)
		{
			float dx = ((float)x - (float)xres / 2.0f) / (float)xres;
			float uvx = dx;
			float dy = ((float)y - (float)yres / 2.0f) / (float)yres;
			//dx *= 2.0f;
			//dy *= 2.0f;
			dx *= 1.3f;
			dy /= 1.3f;

			

			float final_col[3] = { 0,0,0 };
			int final[3] = { 0,0,0 };
			//map.color = col;
			//sphere<float> bx = sphere<float>(boxpos, 1.3f, col);
			for (size_t samp = 0; samp < num_samples; samp++) {
				vec3<float> dir = vec3<float>(dx, 1.f, dy);
				dir.normalize();

				/*float m2 = -30.0f * 3.14159f / 180.0f;
				float l = std::sqrt(dir.e[1]*dir.e[1] + dir.e[2]*dir.e[2]);
				float ddx = dir.e[1] / l;
				float ddy = dir.e[2] / l;
				float ang = (ddy < 0.0) ? 2.0f * 3.14159f - acos(ddx) : acos(ddx);
				ang += m2;
				dir.e[1] = l * cos(ang);
				dir.e[2] = l * sin(ang);*/

				vec3<float> pos = vec3<float>(0.f, -12.f, 0.f);

				ray<float> r = ray<float>(pos, dir, 0.f);

				vec3<float> traced = Trace(map, r, 0,dx, (int)x, (int)y);

				//final_col[0] = final_col[0] + (intensity[0] - final_col[0]) * (1. / (float)(samp + 1));
				//final_col[1] = final_col[1] + (intensity[1] - final_col[1]) * (1. / (float)(samp + 1));
				//final_col[2] = final_col[2] + (intensity[2] - final_col[2]) * (1. / (float)(samp + 1));
				final_col[0] += traced.e[0];
				final_col[1] += traced.e[1];
				final_col[2] += traced.e[2];

			}
			final_col[0] = final_col[0] / (float)num_samples;
			final_col[1] = final_col[1] / (float)num_samples;
			final_col[2] = final_col[2] / (float)num_samples;

			vec3<float> fn = vec3<float>(final_col[0], final_col[1], final_col[2]);
			fn = ACESFilm(fn);
			final_col[0] = fn.e[0];
			final_col[1] = fn.e[1];
			final_col[2] = fn.e[2];

			final_col[0] = powf(final_col[0], 1.0 / 2.2f);
			final_col[1] = powf(final_col[1], 1.0 / 2.2f);
			final_col[2] = powf(final_col[2], 1.0 / 2.2f);

			final[0] = (int)(255.f * final_col[0]);
			final[1] = (int)(255.f * final_col[1]);
			final[2] = (int)(255.f * final_col[2]);

			//intensity[0] = intensity[0] / num_samples;
			//intensity[1] = intensity[1] / num_samples;
			//intensity[2] = intensity[2] / num_samples;

			final[0] = (final[0] < 0) ? 0 : (final[0] > 255) ? 255 : final[0];
			final[1] = (final[1] < 0) ? 0 : (final[1] > 255) ? 255 : final[1];
			final[2] = (final[2] < 0) ? 0 : (final[2] > 255) ? 255 : final[2];

			//const int r2 = dx * dx + dy * dy;
			//const int v = (r2 < xres* xres / 16) ? 255 : 0;
											   //blue        //
			*pixel = (255 << 24) + (final[2] << 16) + (final[1] << 8) + final[0];
			keep_track += 1;
			//std::cout << keep_track << std::end;
			//printf("%i  out of   %i\n", keep_track, (xres * yres));
			++pixel;
		}
		printf("row %i completed, out of %i\n", (int)y, (int)yres);
		y = ++g_currentRowIndex;
	}
}

int main()
{
	//std::cout << "Hello World!\n";

	bool success = load_image(hdr, filename, width, height);


	vec3<float> boxpos = vec3<float>(0.f, 4.f, -1.f);
	vec3<float> col = vec3<float>(0.7f, 0.7f, 0.7f);
	vec3<float> col2 = vec3<float>(0.7f, 0.7f, 0.7f);

	vec3<float> s = vec3<float>(1.3f, 1.3f, 1.3f);
	//sphere<float> sph = sphere<float>(spherepos, 1.3f, col);
	all_shapes<float> map;
	
	map.add(make_shared<triangle_obj<float>>(boxpos, 2.0f, col, 0.f, 1.f, "statue.obj", 1.f, false, "cardiff.png", "ccarr.png", false, false));
	//map.add(make_shared<triangle_obj<float>>(boxpos, 2.0f, col, 0.f, 0.1f, "plane.obj", 1.f, false, "ccardiff.png", "ccarr.png", false, false));

	//map.add(make_shared<triangle_obj<float>>(boxpos, 2.0f, col, 0.f, 1.f, "boat.obj", 1.f, false, "crdiff.png", "rvr.png", false, false));
	//map.add(make_shared<triangle_obj<float>>(boxpos, 2.0f, col, 0.f, 0.4f, "wave.obj", 1.f, false, "crdiff.png", "rvr.png", false, true));

	//map.add(make_shared<sphere<float>>(boxpos, 2.0f, col, 0.f,1.f,1.f));
	//map.add(make_shared<box<float>>(vec3<float>(0.f, 4.f, -4.f), vec3<float>(20.3f, 20.3f, 0.3f), vec3<float>(0.9f, 0.9f, 0.9f), 0.f, 1.0f, 1.f));
	//map.add(make_shared<box<float>>(vec3<float>(0.f, 4.f, 7.f), vec3<float>(20.3f, 20.3f, 0.3f), vec3<float>(0.9f, 0.9f, 0.9f), 0.f, 1.0f, 1.f));
	//map.add(make_shared<box<float>>(vec3<float>(-6.f, 4.f, 0.f), vec3<float>(0.3f, 20.3f, 20.3f), vec3<float>(0.1f, 0.9f, 0.1f), 0.f, 1.0f, 1.f));
	//map.add(make_shared<box<float>>(vec3<float>(6.f, 4.f, 0.f), vec3<float>(0.3f, 20.3f, 20.3f), vec3<float>(0.9f, 0.1f, 0.1f), 0.f, 1.0f, 1.f));
	//map.add(make_shared<box<float>>(vec3<float>(0.f, 12.f, 0.f), vec3<float>(20.3f, 0.3f, 20.3f), vec3<float>(0.9f, 0.9f, 0.9f), 0.f, 1.0f, 1.f));
	map.add(make_shared<box<float>>(vec3<float>(-20.f, 0.5f, 0.3f), vec3<float>(0.3f, 12.3f, 12.3f), vec3<float>(0.9f, 0.9f, 0.9f), 4.5f, 1.0f, 1.f));
	////map.add(make_shared<sphere<float>>(vec3<float>(0.f,6.5f,5.3f), 2.3f, col, 4.5f, 0.3f, 1.f));
	//
	 //map.add(make_shared<box<float>>(vec3<float>(0.f, -10.f, 0.f), vec3<float>(20.3f, 0.3f, 20.3f), vec3<float>(0.9f, 0.9f, 0.9f), 0.f, 1.0f, 1.f));
	//map.add(make_shared<box<float>>(vec3<float>(4.f, 4.f, -3.f), vec3<float>(1.3f, 1.3f, 1.3f), vec3<float>(0.7f, 0.7f, 0.7f), 0.f,1.0f, 1.f));
	//map.add(make_shared<box<float>>(vec3<float>(-3.f, 4.f, -3.f), vec3<float>(1.3f, 1.3f, 2.3f), vec3<float>(0.7f, 0.7f, 0.7f), 0.f, 1.0f, 1.f));


	const size_t numThreads = FORCE_SINGLE_THREAD() ? 1 : std::thread::hardware_concurrency();
	printf("Using %zu threads.\n", numThreads);


	if (numThreads > 1) {
		std::vector<std::thread> threads;
		threads.resize(numThreads);

		for (std::thread& t : threads)
			t = std::thread(thread_func, std::ref(map));

		for (std::thread& t : threads)
			t.join();
	}
	// else if single threaded, just call the rendering function from the main thread
	else {
		thread_func(map);
	}

	
	stbi_write_png("blahhhh.png", xres, yres, 4, &image[0], xres * 4);

}
