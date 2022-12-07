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
 * \brief Perspective camera with depth of field
 *
 * This class implements a simple perspective camera model. It uses an
 * infinitesimally small aperture, creating an infinite depth of field.
 */
class AdvancedCamera : public Camera {
public:
    AdvancedCamera(const PropertyList &propList) : 
        simpleWeighting(propList.getBoolean("simpleWeighting", true)) {

        /* Width and height in pixels. Default: 720p */
        m_outputSize.x() = propList.getInteger("width", 1280);
        m_outputSize.y() = propList.getInteger("height", 720);

        /* Specifies an optional camera-to-world transformation. Default: none */
        m_cameraToWorld = propList.getTransform("toWorld", Transform());

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
            const Point2f &apertureSample) const {

        return Color3f(1.0f);
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
            "  Element Interfaces = %s,\n"
            "  Exit Pupil Bounds = %s,\n"
            "]",
            indent(std::string(m_elementInterfaces.begin(), m_elementInterfaces.end()), 18),
            std::string(m_exitPupilBounds.begin(), m_exitPupilBounds.end())
        );
    }
private:
    // Define a lens element structure (based on PBRT)
    struct LensElementInterface {
        float curvatureRadius;
        float thickness;
        float eta;
        float apertureRadius;
    };

    //====================== ADVANCED CAMERA FIELDS ===================
    const bool simpleWeighting;
    std::vector<LensElementInterface> m_elementInterfaces;
    std::vector<BoundingBox2f> m_exitPupilBounds;
    Transform m_cameraToWorld;
    //=================================================================

    //=============== ADVANCED CAMERA PRIVATE METHODS =================
    // Z-coordinate of the rear lens in our system
    float LensRearZ() const { 
        return m_elementInterfaces.back().thickness; 
    }

    // Z-coordinate of the front lens in our system
    float LensFrontZ() const {
        float zSum = 0;
        for (const LensElementInterface &element : m_elementInterfaces)
            zSum += element.thickness;
        return zSum;
    }

    // The aperture radius of the rear lens in our system
    float RearElementRadius() const {
        return m_elementInterfaces.back().apertureRadius;
    }

    bool TraceLensesFromFilm(const Ray3f &ray, Ray3f *rOut) const {
       
    }

    static bool IntersectSphericalElement(float radius, float zCenter, const Ray3f &ray, float *t, Normal3f *n) {
        //TODO: Compute Intersection with spherical element
    }
    bool TraceLensesFromScene(const Ray3f &rCamera, Ray3f *rOut) const {
        //TODO:
    }
    void DrawLensSystem() const;
    void DrawRayPathFromFilm(const Ray3f &r, bool arrow,
                            bool toOpticalIntercept) const;
    void DrawRayPathFromScene(const Ray3f &r, bool arrow,
                            bool toOpticalIntercept) const;
    static void ComputeCardinalPoints(const Ray3f &rIn, const Ray3f &rOut, float *p,
                                    float *f);
    void ComputeThickLensApproximation(float pz[2], float f[2]) const;
    float FocusThickLens(float focusDistance);
    float FocusBinarySearch(float focusDistance);
    float FocusDistance(float filmDist);
    BoundingBox2f BoundExitPupil(float pFilmX0, float pFilmX1) const;
    void RenderExitPupil(float sx, float sy, const char *filename) const;
    Point3f SampleExitPupil(const Point2f &pFilm, const Point2f &lensSample,
                            float *sampleBoundsArea) const;
    void TestExitPupilBounds() const;
    //========================================================================

};

NORI_REGISTER_CLASS(AdvancedCamera, "advancedCamera");
NORI_NAMESPACE_END
