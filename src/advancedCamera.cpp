/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob, Romain Prévost

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

#include <nori/camera.h>
#include <nori/rfilter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

/**
 * \brief Perspective camera augmented with the thin lens model for depth of field
 * as well as barrel distortion and chromatic abberation to simulate a more realistic camera
 * This class implements a simple perspective camera using the thin lens model. It uses an
 * user defined aperture, creating a finite depth of field.
 */
class AdvancedCamera : public Camera {
public:
    AdvancedCamera(const PropertyList &propList) {
        /* Width and height in pixels. Default: 720p */
        m_outputSize.x() = propList.getInteger("width", 1280);
        m_outputSize.y() = propList.getInteger("height", 720);
        m_invOutputSize = m_outputSize.cast<float>().cwiseInverse();

        /* Specifies an optional camera-to-world transformation. Default: none */
        m_cameraToWorld = propList.getTransform("toWorld", Transform());

        /* Horizontal field of view in degrees */
        m_fov = propList.getFloat("fov", 30.0f);

        /* Near and far clipping planes in world-space units */
        m_nearClip = propList.getFloat("nearClip", 1e-4f);
        m_farClip = propList.getFloat("farClip", 1e4f);

        /* Get thin lens parameters: lens radius and focal distance */
        m_focalDistance = propList.getFloat("focalDist", 1.0f);
        m_lensRadius = propList.getFloat("lensRadius", 0.0f);

        /* Get lens distortion and chromatic aberration parameters */
        m_distortion = propList.getVector2("distortion", Vector2f::Zero());
        m_chromaticStrength = propList.getVector3("chromaticAberation", Vector3f::Zero());

        m_rfilter = NULL;
    }
    /**
     * \brief Perform some action associated with the object
     *
     * The default implementation throws an exception. Certain objects
     * may choose to override it, e.g. to implement initialization, 
     * testing, or rendering functionality.
     *
     * This function is called by the XML parser once it has
     * constructed an object and added all of its children
     * using \ref addChild().
     */
    virtual void activate() override {
        float aspect = m_outputSize.x() / (float) m_outputSize.y();

        /* Project vectors in camera space onto a plane at z=1:
         *
         *  xProj = cot * x / z
         *  yProj = cot * y / z
         *  zProj = (far * (z - near)) / (z * (far-near))
         *  The cotangent factor ensures that the field of view is 
         *  mapped to the interval [-1, 1].
         */
        float recip = 1.0f / (m_farClip - m_nearClip),
              cot = 1.0f / std::tan(degToRad(m_fov / 2.0f));

        Eigen::Matrix4f perspective;
        perspective <<
            cot, 0,   0,   0,
            0, cot,   0,   0,
            0,   0,   m_farClip * recip, -m_nearClip * m_farClip * recip,
            0,   0,   1,   0;

        /**
         * Translation and scaling to shift the clip coordinates into the
         * range from zero to one. Also takes the aspect ratio into account.
         */
        m_sampleToCamera = Transform( 
            Eigen::DiagonalMatrix<float, 3>(Vector3f(0.5f, -0.5f * aspect, 1.0f)) *
            Eigen::Translation<float, 3>(1.0f, -1.0f/aspect, 0.0f) * perspective).inverse();

        /* If no reconstruction filter was assigned, instantiate a Gaussian filter */
        if (!m_rfilter) {
            m_rfilter = static_cast<ReconstructionFilter *>(
                    NoriObjectFactory::createInstance("gaussian", PropertyList()));
            m_rfilter->activate();
        }
    }

    /**
     * \brief Importance sample a ray according to the camera's response function
     *
     * \param ray
     *    A ray data structure to be filled with a position 
     *    and direction value
     *
     * \param samplePosition (pFilm)
     *    Denotes the desired sample position on the film
     *    expressed in fractional pixel coordinates
     *
     * \param apertureSample (pLens)
     *    A uniformly distributed 2D vector that is used to sample
     *    a position on the aperture of the sensor if necessary.
     *
     * \return
     *    An importance weight associated with the sampled ray.
     *    This accounts for the difference in the camera response
     *    function and the sampling density.
     */
    Color3f sampleRay(Ray3f &ray,
            const Point2f &samplePosition,
            const Point2f &apertureSample,
            int channel) const {

        /* Compute the corresponding position on the 
            near plane (in local camera space) */
        Point3f nearP = m_sampleToCamera * Point3f(
        samplePosition.x() * m_invOutputSize.x(),
        samplePosition.y() * m_invOutputSize.y(), 0.0f);

        Vector3f d = nearP.normalized();

        // Check for lens distortion
        if(!m_distortion.isZero()) {
            #define F_EPSILON 1e-6
            // Compute the distortion using the same method as in Mitsuba
            // i.e. Update the position of the ray on the near plane using 
            // using the given distortion components
            float y = Vector2f(nearP.x() / nearP.z(), nearP.y() / nearP.z()).norm();
            float r = y;
            float r2, f, df;
            int i = 0;
            // Solve for distortion
            while(true) {
                r2 = r * r;
                f = r * (1 + (m_distortion.x() * r2) + m_distortion.y() * (r2 * r2)) - y;
                df = 1 + (3 * m_distortion.x() * r2) + (5 * m_distortion.y() * r2 * r2);

                r = r - f / df;
                if(std::abs(f) < F_EPSILON || i++ > 4) {
                    break;
                }
            } 

            float distortionFactor = r/y;

            // Update the near plane position
            nearP.x() *= distortionFactor;
            nearP.y() *= distortionFactor;
            d = nearP.normalized();
        }

        float w = 0.0f; // Weight of the current channel
        Color3f color(1.0f); // Output color
        #define N_CHANNELS 3 // Red(0), Green(1), Blue(2)

        // Check for chromatic aberration
        if(!m_chromaticStrength.isZero()) { 
            // Sanity check
            assert(0 <= channel && channel < N_CHANNELS);
            w = m_chromaticStrength[channel];
            color = Color3f(0.0f); // Reset color channels
            color[channel] = 1.0f; // Only activate currently sampled channel
        }

        /* Turn into a normalized ray direction, and
            adjust the ray interval accordingly */
        float invZ = 1.0f / d.z();

        ray.o = Point3f(0.0f);
        ray.d = d;
        
        // Take into account DOF if needed with aberration sampling offset
        if(m_lensRadius > 0.0f || !m_chromaticStrength.isZero()) {
            // Sample point on lens
            Point2f pLens = m_lensRadius * 
                Warp::squareToUniformDisk(apertureSample);

            // Compute point on plance of focus
            float ft = m_focalDistance / ray.d.z();
            Point3f pFocus = ray(ft);

            // Update the sample position for chromatic aberration
            Point2f sp(
                samplePosition.x() - (0.5f * m_outputSize.x()),
                samplePosition.y() - (0.5f * m_outputSize.y())
            );
            sp /= m_outputSize.maxCoeff();

            Point2f deltaSP = sp * sp.squaredNorm() * w;
            pFocus = pFocus + Point3f(-deltaSP.x(), deltaSP.y(), 0.0f);

            // Update ray for DOF effect given by the thin lens model
            ray.o = Point3f(pLens.x(), pLens.y(), 0.0f);
            d = (pFocus - ray.o).normalized();
            ray.o = m_cameraToWorld * ray.o;
            
            ray.d = m_cameraToWorld * d;
        } else {
            ray.o = m_cameraToWorld * Point3f(0, 0, 0);
            ray.d = m_cameraToWorld * d;
        }
        
        // Update mint and maxt
        ray.mint = m_nearClip * invZ;
        ray.maxt = m_farClip * invZ;
        ray.update();

        return color;
    }

    bool hasChromaticAberrations() const {
        return !m_chromaticStrength.isZero();
    }

    virtual void addChild(NoriObject *obj) override {
        switch (obj->getClassType()) {
            case EReconstructionFilter:
                if (m_rfilter)
                    throw NoriException("Camera: tried to register multiple reconstruction filters!");
                m_rfilter = static_cast<ReconstructionFilter *>(obj);
                break;

            default:
                throw NoriException("Camera::addChild(<%s>) is not supported!",
                    classTypeName(obj->getClassType()));
        }
    }

    /// Return a human-readable summary
    virtual std::string toString() const override {
        return tfm::format(
            "AdvancedCamera[\n"
            "  cameraToWorld = %s,\n"
            "  outputSize = %s,\n"
            "  fov = %f,\n"
            "  clip = [%f, %f],\n"
            "  rfilter = %s\n"
            "]",
            indent(m_cameraToWorld.toString(), 18),
            m_outputSize.toString(),
            m_fov,
            m_nearClip,
            m_farClip,
            indent(m_rfilter->toString())
        );
    }
private:
    Vector2f m_invOutputSize;
    Transform m_sampleToCamera;
    Transform m_cameraToWorld;
    float m_fov;
    float m_nearClip;
    float m_farClip;

protected:
    float m_lensRadius;
    float m_focalDistance;
    Vector2f m_distortion;        //Parameters for barrel distortion (as in mitsuba)
    Vector3f m_chromaticStrength; //Strength of chromatic aberration along each color component

};

NORI_REGISTER_CLASS(AdvancedCamera, "advancedCamera");
NORI_NAMESPACE_END
