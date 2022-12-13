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

#include <nori/object.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

template <typename T>
class Checkerboard : public Texture<T> {
public:
    Checkerboard(const PropertyList &props);

    virtual std::string toString() const override;

    virtual T eval(const Point2f & uv) const override {

        int x = abs(std::floor(uv.x() / m_scale.x() - m_delta.x()));
        int y = abs(std::floor(uv.y() / m_scale.y() - m_delta.y()));
        
        return x % 2 == y % 2 ? m_value1 : m_value2;
    }

protected:
    T m_value1;
    T m_value2;

    Point2f m_delta;
    Vector2f m_scale;
};

template <>
Checkerboard<float>::Checkerboard(const PropertyList &props) {
    m_delta = props.getPoint2("delta", Point2f(0));
    m_scale = props.getVector2("scale", Vector2f(1));
    m_value1 = props.getFloat("value1", 0.f);
    m_value2 = props.getFloat("value2", 1.f);
}

template <>
Checkerboard<Color3f>::Checkerboard(const PropertyList &props) {
    m_delta = props.getPoint2("delta", Point2f(0));
    m_scale = props.getVector2("scale", Vector2f(1));
    m_value1 = props.getColor("value1", Color3f(0));
    m_value2 = props.getColor("value2", Color3f(1));
}


template <>
std::string Checkerboard<float>::toString() const {
    return tfm::format(
        "Checkerboard[\n"
                "  delta = %s,\n"
                "  scale = %s,\n"
                "  value1 = %f,\n"
                "  value2 = %f,\n"
                "]",
        m_delta.toString(),
        m_scale.toString(),
        m_value1,
	    m_value2
    );
}

template <>
std::string Checkerboard<Color3f>::toString() const {
    return tfm::format(
        "Checkerboard[\n"
                "  delta = %s,\n"
                "  scale = %s,\n"
                "  tex1 = %s,\n"
                "  tex2 = %s,\n"
                "]",
        m_delta.toString(),
        m_scale.toString(),
        m_value1.toString(),
	    m_value2.toString()
    );
}

NORI_REGISTER_TEMPLATED_CLASS(Checkerboard, float, "checkerboard_float")
NORI_REGISTER_TEMPLATED_CLASS(Checkerboard, Color3f, "checkerboard_color")
NORI_NAMESPACE_END