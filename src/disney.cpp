/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <bits/stdc++.h>

NORI_NAMESPACE_BEGIN

inline float SchlickFresnel(const float u)
{
    float m = clamp(1-u, 0.0f, 1.0f);
    return pow(m,5);
}

inline float GGX(const float NdotV, const float alphaG) 
{
    float a = alphaG*alphaG;
    float b = NdotV*NdotV;
    return 1 / (NdotV + sqrt(a + b - a*b));
}

template <typename T> 
inline T lerp(const T &a,const T &b, cons T& t) {
    return t*a + (1.0-t)*b;
}


class Disney : public BSDF {
public:
    Disney(const PropertyList &propList) {

        this->m_metallic = propList.getFloat("metallic", 0.0f);
        this->m_specular = propList.getFloat("specular", 0.0f);
        this->m_roughness = propList.getFloat("roughness", 0.0f);
        this->m_sheen = propList.getFloat("sheen", 0.0f);

        this->m_baseColor = propList.getColor("baseColor",Color3f(0.0f));
        this->m_sheenTint = propList.getColor("sheenTint",Color3f(0.0f));
    }

    /// Evaluate the BRDF for the given pair of directions
    virtual Color3f eval(const BSDFQueryRecord &bRec) const override {

        float NdotV = Frame::cosTheta(bRec.wi);
        float NdotL = Frame::cosTheta(bRec.wo);

        if(NdotV < 0 || NdotL < 0) return Color3f(0.0f);
        Vector3f wh = (bRec.wi + bRec.wo).normalized();

        float LdotH = bRec.wo.dot(wh);
        float VdotH = bRec.wi.dot(wh);
        float NdotH = Frame::cosTheta(wh);
        
        // Base color part
        float luminance = m_baseColor.getLuminance();
        Color3f Ctint = (luminance > 0.f) ? Color3f(m_baseColor.r() / luminance, m_baseColor.g() / luminance, m_baseColor.b() / luminance) : Color3f(1.0f);
        Color3f CtintMix = 0.08 * m_specular * lerp(Ctint,m_specularTint, Color3f(1));
        Color3f Cspec = lerp(CtintMix, m_baseColor,m_metallic);
        Color3f Csheen = lerp(Ctint,Csheen,1.0f);

        // Diffuse part
        float fd90 = 0.5 + 2 * m_roughness * pow(bRec.wi.dot(wh),2);
        float fl = SchlickFresnel(NdotL);
        float fv = SchlickFresnel(NdotV);
        Color3f diffuse = m_baseColor * INV_PI * (1.f + (fd90 - 1.f) * fl) * (1.f + (fd90 - 1.f) * fv);

        // Specular part
        float Ds = Warp::squareToBeckmannPdf(wh,pow(m_roughness,2));
        float FH = SchlickFresnel(LdotH);
        Color3f Fs = lerp(Color3f(0.0f),Color3f(1.0f),FH);
        float Gs = GGX(NdotL,FIXED_ROUGHNESS);

        Color3f specular = Gs*Fs*Ds;

        // Sheen part
        float luminance = 0.3 * m_baseColor.x() + 0.6f * m_baseColor.y() + 1.0f * m_baseColor.z();
        Color3f tint = (luminance > 0.0f) ? (1.0f / luminance) * m_baseColor : Color3f(1.0f);
        Color3f Fsheen = FH * m_sheen * lerp(tint,m_sheenTint,1.0f);
        
        return (1 - m_metallic) * (diffuse + Fsheen) +  specular;
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    virtual float pdf(const BSDFQueryRecord &bRec) const override {
        return 1.0f;
    }

    /// Sample the BRDF
    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const override {
        return Color3f(0.0f);
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Disney[\n"
            "  baseColor = %f,\n"
            "  metallic = %f,\n"
            "  specular = %f,\n"
            "  specularTint = %f,\n"
            "  roughness = %f,\n"
            "  sheen = %f\n"
            "  sheenTint = %f\n"
            "]",
            m_baseColor,
            m_metallic,
            m_specular,
            m_specularTint,
            m_roughness,
            m_sheen,
            m_sheenTint
        );
    }
private:
    float m_metallic;
    float m_specular;
    float m_roughness;
    float m_sheen;

    Color3f m_baseColor;
    Color3f m_specularTint;
    Color3f m_sheenTint;

    const float FIXED_ROUGHNESS = 0.25f;
};

NORI_REGISTER_CLASS(Disney, "disney");
NORI_NAMESPACE_END
