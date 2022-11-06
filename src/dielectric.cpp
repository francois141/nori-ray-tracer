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

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    virtual Color3f eval(const BSDFQueryRecord &) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    virtual float pdf(const BSDFQueryRecord &) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {

        float theta = Frame::cosTheta(bRec.wi);
        Vector3f normalVector = Vector3f(0,0,1.0f);
        // Reflected
        if(fresnel(theta,m_extIOR,m_intIOR) > sample.x()) {
            bRec.eta = 1.0f;
            bRec.wo = Vector3f(-bRec.wi.x(),-bRec.wi.y(),bRec.wi.z());
        } 
        // Refracted
        else {
            float factor = m_extIOR / m_intIOR;

            if(theta < 0.0f) {
                factor = 1 / factor;
                normalVector.z() *= -1;   
            }

            Vector3f part1 = -factor*(bRec.wi - bRec.wi.dot(normalVector)*normalVector);
            Vector3f part2 = -normalVector*sqrt(1 - pow(factor,2)*(1 - pow(bRec.wi.dot(normalVector),2)));
            Vector3f wo = (part1 + part2).normalized();

            bRec.wo = wo;
            bRec.eta = m_extIOR / m_intIOR;
        }
        bRec.measure = EDiscrete;

        return 1.0f;
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
   
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
