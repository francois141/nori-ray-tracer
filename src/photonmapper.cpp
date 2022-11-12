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

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class PhotonMapper : public Integrator
{
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    PhotonMapper(const PropertyList &props)
    {
        /* Lookup parameters */
        m_photonCount = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
    }

    virtual void preprocess(const Scene *scene) override
    {
        cout << "Gathering " << m_photonCount << " photons .. ";
        cout.flush();

        /* Create a sample generator for the preprocess step */
        Sampler *sampler = static_cast<Sampler *>(
            NoriObjectFactory::createInstance("independent", PropertyList()));

        /* Allocate memory for the photon map */
        m_photonMap = std::unique_ptr<PhotonMap>(new PhotonMap());
        m_photonMap->reserve(m_photonCount);

        /* Estimate a default photon radius */
        if (m_photonRadius == 0)
            m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;

        /* How to add a photon?
         * m_photonMap->push_back(Photon(
         *	Point3f(0, 0, 0),  // Position
         *	Vector3f(0, 0, 1), // Direction
         *	Color3f(1, 2, 3)   // Power
         * ));
         */

        // Trace each photon
        while (true)
        {

            // Get an emitter
            const Emitter *light = scene->getRandomEmitter(sampler->next1D());

            // Start the tracing
            Ray3f currentRay;
            Color3f power = light->samplePhoton(currentRay, sampler->next2D(), sampler->next2D()) * scene->getLights().size();

            // Continue until the the Russian Roulette breaks
            while (true)
            {

                // Check if there is an intersection
                Intersection its;
                if (!scene->rayIntersect(currentRay, its))
                    break;

                // If we hit a diffuse surface ==> Store the photon
                if (its.mesh->getBSDF()->isDiffuse())
                {
                    m_photonMap->push_back(Photon(its.p, -currentRay.d, power));

                    // Return if the map is full
                    if(m_photonMap->size() == m_photonCount) {
                        return;
                    }               
                }

                // Perform Russian Roulette
                float probability = std::min(power.x(), 0.99f);
                if (sampler->next1D() > probability)
                {
                    break;
                }
                power /= probability;

                // Sample the brdf
                BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
                Color3f brdf = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
                power *= brdf;

                // Continue the recursion
                currentRay = Ray3f(its.p, its.toWorld(bRec.wo));
            }
        }

        /* Build the photon map */
        m_photonMap->build();
    }

    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const override
    {

        /* How to find photons?
         * std::vector<uint32_t> results;
         * m_photonMap->search(Point3f(0, 0, 0), // lookup position
         *                     m_photonRadius,   // search radius
         *                     results);
         *
         * for (uint32_t i : results) {
         *    const Photon &photon = (*m_photonMap)[i];
         *    cout << "Found photon!" << endl;
         *    cout << " Position  : " << photon.getPosition().toString() << endl;
         *    cout << " Power     : " << photon.getPower().toString() << endl;
         *    cout << " Direction : " << photon.getDirection().toString() << endl;
         * }
         */

        // Output color & attenuation of the russian roulette
        Color3f color = BLACK;
        Color3f attenuation = WHITE;

        // This ds stores the recursive ray
        Ray3f currentRay = _ray;

        // Continue until the Russian Roulette says stop
        while (true)
        {
            Intersection its;
            // If the ray has no intersection we can return the black color;
            if (!scene->rayIntersect(currentRay, its))
                return color;

            // We add the Le part to the record if the mesh is an emitter
            if (its.mesh->isEmitter())
            {
                EmitterQueryRecord rec(currentRay.o, its.p, its.shFrame.n);
                color += attenuation * its.mesh->getEmitter()->eval(rec);
            }

            // If we are facing a diffuse surface, we can return the value given by the photons
            if (its.mesh->getBSDF()->isDiffuse())
            {

                // Retrieve the photon list
                std::vector<uint32_t> results;
                m_photonMap->search(its.p, m_photonRadius, results);
                Color3f photonColor = BLACK;

                // Compute value for each photon
                for (auto idx : results)
                {
                    const Photon &p = (*m_photonMap)[idx];
                    BSDFQueryRecord bRec(its.shFrame.toLocal(-currentRay.d), its.shFrame.toLocal(p.getDirection()), ESolidAngle);
                    photonColor += its.mesh->getBSDF()->eval(bRec) * p.getPower();
                }

                // Return estimated density
                return color + attenuation * (photonColor * INV_PI / (m_photonRadius * m_photonRadius * m_photonCount));
            }

            // Update the russian roulette
            float probability = std::min(attenuation.x(), 0.99f);
            if (sampler->next1D() > probability)
            {
                return color;
            }
            attenuation /= probability;

            // Sample the BRDF
            BSDFQueryRecord bRec(its.shFrame.toLocal(-currentRay.d));
            Color3f brdf = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            attenuation *= brdf;

            // Continue the recursion
            currentRay = Ray3f(its.p, its.toWorld(bRec.wo));
        }
    }

    virtual std::string toString() const override
    {
        return tfm::format(
            "PhotonMapper[\n"
            "  photonCount = %i,\n"
            "  photonRadius = %f\n"
            "]",
            m_photonCount,
            m_photonRadius);
    }

private:
    /*
     * Important: m_photonCount is the total number of photons deposited in the photon map,
     * NOT the number of emitted photons. You will need to keep track of those yourself.
     */
    int m_photonCount;
    float m_photonRadius;
    std::unique_ptr<PhotonMap> m_photonMap;

    const Color3f BLACK = Color3f(0.0f);
    const Color3f WHITE = Color3f(1.0f);
};

NORI_REGISTER_CLASS(PhotonMapper, "photonmapper");
NORI_NAMESPACE_END
