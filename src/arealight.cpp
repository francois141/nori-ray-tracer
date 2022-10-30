/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/shape.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter {
public:
    AreaEmitter(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }

    virtual std::string toString() const override {
        return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                "]",
                m_radiance.toString());
    }

    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        return lRec.n.dot(-lRec.wi) > 0.0f ? m_radiance : BLACK;
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        ShapeQueryRecord sRec(lRec.ref);
        m_shape->sampleSurface(sRec, sample);

        lRec.p = sRec.p;
        lRec.n = sRec.n;
        lRec.wi = (lRec.p - lRec.ref).normalized();
        lRec.shadowRay = Ray3f(lRec.ref,lRec.wi,Epsilon,(lRec.p - lRec.ref).norm() - Epsilon);
        lRec.pdf = pdf(lRec);
        
        Color3f evaluation = eval(lRec) / pdf(lRec);

        return lRec.pdf > 0.0f ? evaluation : BLACK;
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        float angle = lRec.n.dot(-lRec.wi);

        if(angle > 0.0f) {
            ShapeQueryRecord rec(lRec.ref,lRec.p);
            return m_shape->pdfSurface(rec) * (rec.p - rec.ref).squaredNorm() / angle;
        }

        return 0.0f;
    }

    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
        throw NoriException("To implement...");
    }


protected:
    Color3f m_radiance;
    const Color3f BLACK = Color3f(0.0f);
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END 
