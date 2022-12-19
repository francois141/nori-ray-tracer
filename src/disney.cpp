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

// Implementation from : https://thebookofshaders.com/glossary/?search=mix
template <typename T1,typename T2> 
inline T1 lerp(const T2 &t,const T1 &a,const T1 &b) {
    return (1.0f-t)*a + t*b;
}

class Disney : public BSDF {
public:
    Disney(const PropertyList &propList) {

        // Retrieve the differents parameters
        this->m_metallic = propList.getFloat("metallic", 0.0f);
        this->m_specular = propList.getFloat("specular", 0.0f);
        this->m_roughness = propList.getFloat("roughness", 0.0f);
        this->m_sheen = propList.getFloat("sheen", 0.0f);
        this->m_sheenTint = propList.getFloat("sheenTint",0.0f);
        this->m_specularTint = propList.getFloat("specularTint",0.0f);

        this->m_baseColor = propList.getColor("baseColor",Color3f(0.0f));

        this->m_alpha = std::max(1e-3, std::pow(m_roughness,2));
    }

    /// Evaluate the BRDF for the given pair of directions
    virtual Color3f eval(const BSDFQueryRecord &bRec) const override {

        // Basic vector computations
        float NdotV = Frame::cosTheta(bRec.wi);
        float NdotL = Frame::cosTheta(bRec.wo);

        if(NdotV < 0 || NdotL < 0) 
            return BLACK;

        Vector3f wh = (bRec.wi + bRec.wo).normalized();

        float LdotH = bRec.wo.dot(wh);
        float VdotH = bRec.wi.dot(wh);
        float NdotH = Frame::cosTheta(wh);
        
        // Base color part
        float luminance = m_baseColor.getLuminance();
        Color3f Ctint = (luminance > 0.f) ? Color3f(m_baseColor.r() / luminance, m_baseColor.g() / luminance, m_baseColor.b() / luminance) : Color3f(1.0f);
        Color3f CtintMix = 0.08 * m_specular * lerp(m_specularTint, WHITE, Ctint);
        Color3f Cspec = lerp(m_metallic, CtintMix, m_baseColor);
        Color3f Csheen = lerp(m_sheenTint, WHITE , Ctint);

        // Diffuse part
        float fd90 = 0.5 + 2 * m_roughness * pow(VdotH,2);
        float fl = SchlickFresnel(NdotL);
        float fv = SchlickFresnel(NdotV);
        Color3f diffuse = m_baseColor * INV_PI * (1.f + (fd90 - 1.f) * fl) * (1.f + (fd90 - 1.f) * fv);

        // Specular part
        float alpha = std::max(0.001f, m_roughness * m_roughness);
        float Ds = Warp::squareToGTR2Pdf(wh,alpha);
        float FH = SchlickFresnel(LdotH);
        Color3f Fs = lerp(FH,Cspec, WHITE);
        float Gs = GGX(NdotL,alpha) * GGX(NdotV, alpha);

        Color3f specular = Gs*Fs*Ds;

        // Sheen part
        Color3f Fsheen = FH * m_sheen * lerp(m_sheenTint,WHITE, Ctint);
        return (1 - m_metallic) * (diffuse + Fsheen) + specular;
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    virtual float pdf(const BSDFQueryRecord &bRec) const override {

    	float cosinus = Frame::cosTheta(bRec.wo);
        if(cosinus <= 0.0f) {
            return 0.0f;
        }

        Vector3f normal = (bRec.wi + bRec.wo).normalized();
        
        float metallicTerm = Warp::squareToGTR2Pdf(normal, m_alpha) * Frame::cosTheta(normal) / (4.0f*abs(normal.dot(bRec.wo)));
        float diffuseTerm = cosinus*INV_PI;

        return (1-m_metallic) * diffuseTerm + m_metallic * metallicTerm; 
    }

    /// Sample the BRDF
    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const override {
        
        if(Frame::cosTheta(bRec.wi) <= 0.0f) {
            return BLACK;
        }
            
        if(_sample.x() <= m_metallic) {
            // Not diffuse part
            Point2f rescaledSample = Point2f(_sample.x() / m_metallic,_sample.y());
            Vector3f normal = Warp::squareToGTR2(rescaledSample,m_alpha);
            bRec.wo = ((2.0f * bRec.wi.dot(normal) * normal) - bRec.wi).normalized();
        } else {
            // Diffuse part
            Point2f rescaledSample = Point2f((_sample.x()-m_metallic) / (1 - m_metallic),_sample.y());
            bRec.wo = Warp::squareToCosineHemisphere(rescaledSample);
        }
        
        float cosTheta = Frame::cosTheta(bRec.wo);
        if(cosTheta <= 0.0f) {
            return BLACK;
        }
            
        return eval(bRec) * cosTheta / pdf(bRec);
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Disney[\n"
            "  baseColor = %s,\n"
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

    void addChild(NoriObject *obj) override {}

private:
    float m_metallic;
    float m_specular;
    float m_roughness;
    float m_sheen;
    float m_alpha;

    Color3f m_baseColor;
    float m_specularTint;
    float m_sheenTint;

    const Color3f BLACK = Color3f(0.0f);
    const Color3f WHITE = Color3f(1.0f);
};

NORI_REGISTER_CLASS(Disney, "disney");
NORI_NAMESPACE_END
