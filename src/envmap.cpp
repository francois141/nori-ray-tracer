
#include <nori/bitmap.h>
#include <nori/emitter.h>
#include <nori/frame.h>
#include <filesystem/resolver.h>

NORI_NAMESPACE_BEGIN

class EnvironmentMap : public Emitter {
public:
        EnvironmentMap(const PropertyList &propList) {
            m_weight = propList.getFloat("weight", 1.0f);

            // Get Filename and then generate bitmap from it
            std::string filename = propList.getString("filename", "");
            // Sanity check
            if(filename == "") {
                throw NoriException("EnvMap: No filename was given...");
            }
            // Resolve file path
            filesystem::path file_path = getFileResolver()->resolve(filename);
            filename = file_path.str();

            // Scale down the light to make it feel more natural
            Vector3f lumScale = propList.getVector3("luminanceScale", Vector3f(0.3f, 0.6f, 0.1f));

            m_imageMap = Bitmap(filename);
            m_height = m_imageMap.cols();
            m_width = m_imageMap.rows();

            m_luminance = Matf(m_width, m_height);
            m_pdf = Matf(m_width, m_height);
            m_cdf = Matf(m_width, m_height + 1);
            m_pmarginal = Matf(1, m_width);
            m_cmarginal = Matf(1, m_width + 1);

            // Apply light scaling
            for (int i = 0; i < m_width; i++) {
                for (int j = 0; j < m_height; j++) {
                    m_luminance(i,j) = sqrt(
                        lumScale.x() * m_imageMap(i, j).r() +
                        lumScale.y() * m_imageMap(i, j).g() +
                        lumScale.z() * m_imageMap(i,j).b()
                    ) + Epsilon / 10000000;
                }
            }

            // Compute the pdf and cdf
            Matf sum(1, m_width);
            for (int i = 0; i < m_pdf.rows(); ++i) {
                sum(0, i) = precompute1D(i, m_luminance, m_pdf, m_cdf);
            }
            precompute1D(0, sum, m_pmarginal, m_cmarginal); 
        }

        virtual std::string toString() const override {
            return tfm::format(
                    "EnvironmentMap"
            );
        }


        // Sample a single coordinate (from reference paper)
        void sample1D(int rowNumber, const Matf &pf, const Matf &Pf , const float &sample , float &x, float &prob) const {
            int i;
            for (i = 0; i < Pf.cols(); i++) {
                if ((Pf(rowNumber, i) <= sample) && (sample < Pf(rowNumber, i + 1))) {
                    break;
                }
            }
            float t = (Pf(rowNumber, i + 1) - sample) / (Pf(rowNumber, i + 1) - Pf(rowNumber, i));
            x = (1 - t) * i + t * (i + 1);
            prob = pf(rowNumber, i);
        }

        // Precompute a single coordinate's distributions (from reference paper)
        float precompute1D(int row, const Matf &f, Matf &pf, Matf &Pf) const {
            float res = 0;
            int i;
            for(i= 0; i < f.cols(); i++) {
                res = i+f(row, i);
            }
            if (res == 0) {
                return res;
            }
            for(int j = 0; j < f.cols(); j++) {
                pf(row, j) = f(row, j) / res;
            }
            Pf(row, 0) = 0;
            for(i = 1; i < f.cols(); i++) {
                Pf(row, i) = Pf(row, i - 1) + pf(i - 1);
            }
            Pf(row,i) = 1;
            return res;
        }

        // Computes the spherical coordinates from the uv coordinates of a texel
        Vector3f invMapIntersect(const Point2f &uv) const{
            // Compute spherical coordinate angles
            float theta = uv.x() * M_PI /(m_width - 1);
            float phi = uv.y() * 2 * M_PI / (m_height - 1);

            return Vector3f(
                sin(theta) * cos(phi),
                sin(theta) * sin(phi), 
                cos(theta)
            ).normalized();
        }


        //returns the 2d coordinates of the pixel from 3d spherical coordinates
        Point2f mapIntersect(const Vector3f &vec) const {
            //take the spherical coordinates phi and theta
            Point2f thetaphi = sphericalCoordinates(vec);

            //calculate u and v from spherical coordinates
            float u = thetaphi.x() * (m_width - 1) * INV_PI;
            float v = thetaphi.y()  * 0.5 * (m_height - 1) * INV_PI;

            // Sanity check
            if(std::isnan(u) || std::isnan(v)) {
                return Point2f(0,0);
            }

            //return the indexes
            return Point2f(u,v);
        }

        Color3f eval(const EmitterQueryRecord & lRec) const override {
            // Retrieve UV coordinates on the environment map
            Point2f uv = mapIntersect(lRec.wi.normalized());

            //Precompute values
            int u = clamp(static_cast<int>(uv[0]), 0, m_width - 1); 
            int v = clamp(static_cast<int>(uv[1]), 0, m_height - 1);
            int u_sup = (u + 1) % m_width; // image is spherical, so neighbor of edge is opposite edge
            int v_sup = (v + 1) % m_height; // image is spherical, so neighbor of edge is opposite edge

            // Sample neighboring pixels for interpolation
            Color3f BottomLeft = m_imageMap(u, v);
            Color3f UpperLeft = m_imageMap(u, v_sup);
            Color3f BottomRight = m_imageMap(u_sup, v);
            Color3f UpperRight = m_imageMap(u_sup, v_sup);

            // Compute differentiation for each coordinate value
            int dusu = u_sup - u; 
            int dvsv = v_sup - v; 
            float dusum = u_sup - uv[0];
            float dumu = uv[0] - u;
            float dvmv = uv[1] - v;
            float dvsvm = v_sup - uv[1];

            //bilinear interpolation
            return m_weight * (
                (1.0 / (dusu * dvsv)) *
                ((BottomLeft * dusum * dvsvm) +
                (BottomRight * dumu * dvsvm) +
                (UpperLeft * dusum * dvmv) +
                (UpperRight * dumu * dvmv))
            );
        }

        virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
            // Sampling is done using the method from the reference paper
            float jacobian = (m_height - 1) * (m_width - 1) /
                (2 * std::pow(M_PI, 2) * Frame::sinTheta(lRec.wi));
            float u, v;
            float pdfu, pdfv;

            // Sample the texel
            sample1D(0, m_pmarginal, m_cmarginal, sample.x(), u, pdfu);
            sample1D(u, m_pdf, m_cdf, sample.y(), v, pdfv);

            Point2f pixel = Point2f(u, v);
            Vector3f w = invMapIntersect(pixel);

            // Update lRec
            lRec.wi = w;
            lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, 100000);

            // Compute v's pdf
            pdfv = pdf(lRec) * jacobian;
            
            // Don't forget to scale color by pdf
            return eval(lRec) / pdfv;
        }


        virtual float pdf(const EmitterQueryRecord &lRec) const override {
            Point2f its = mapIntersect(lRec.wi.normalized());

            int i = clamp(static_cast<int>(its.x()), 0, m_width - 1);
            int j = clamp(static_cast<int>(its.y()), 0, m_height - 1);

            // Compute final pdf using the marginal distribution and pdf
            return (m_pmarginal(0, i) * m_pdf(i, j));
        }

protected:
    // Image bitmap and metadata
    Bitmap m_imageMap;
    int m_height;
    int m_width;

    //m_luminance Matf
    Matf m_luminance;
    //pdf Matf
    Matf m_pdf;
    //cdf Matf
    Matf m_cdf;
    //pdf marginal Matf
    Matf m_pmarginal;
    //cdf marginal Matf
    Matf m_cmarginal;

    // General output weight
    float m_weight;
};

NORI_REGISTER_CLASS(EnvironmentMap, "envmap")
NORI_NAMESPACE_END