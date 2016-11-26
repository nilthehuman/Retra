/*
 * Copyright 2015 Dániel Arató
 *
 * This file is part of Retra.
 *
 * Retra is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Retra is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Retra.  If not, see <http://www.gnu.org/licenses/>.
 */

// Scene class methods
// Part of Retra, the Reference Tracer

#include "scene.h"

#include <math.h>
#include <stdlib.h>
#include <cassert>

#include "ray.h"

namespace Retra {

    void Surface::rotate( Vector& point, double theta, WorldAxis axis )
    {
        double c = cos( theta );
        double s = sin( theta );
        Vector newPoint;
        switch( axis )
        {
            case AXIS_X:
                newPoint.x = point.x;
                newPoint.y = c * point.y - s * point.z;
                newPoint.z = s * point.y + c * point.z;
                break;
            case AXIS_Y:
                newPoint.x =  c * point.x + s * point.z;
                newPoint.y = point.y;
                newPoint.z = -s * point.x + c * point.z;
                break;
            case AXIS_Z:
                newPoint.x = c * point.x - s * point.y;
                newPoint.y = s * point.x + c * point.y;
                newPoint.z = point.z;
                break;
            default:
                assert( false );
        }
        point = newPoint;
    }

    void Light::move( const Vector& translation ) const
    {
        for ( LightPartIt part = partsBegin(); part != partsEnd(); part++ )
            (*part)->move( translation );
        scene->setChanged();
    }
    void Light::move( double theta, WorldAxis axis ) const
    {
        for ( LightPartIt part = partsBegin(); part != partsEnd(); part++ )
            (*part)->move( theta, axis );
        scene->setChanged();
    }

    void Thing::move( const Vector& translation ) const
    {
        for ( ThingPartIt part = partsBegin(); part != partsEnd(); part++ )
            (*part)->move( translation );
        scene->setChanged();
    }
    void Thing::move( double theta, WorldAxis axis ) const
    {
        for ( ThingPartIt part = partsBegin(); part != partsEnd(); part++ )
            (*part)->move( theta, axis );
        scene->setChanged();
    }

    Triplet LightPart::getEmission( const Vector& ) const
    {
        return ((Light*)parent)->getEmission(); // By default
    }

    Triplet LightTriangle::getEmission( const Vector& direction ) const
    {
        double tilt = direction * getNormal( Vector::Zero );
        if ( parent->isBackCulled() && tilt < 0 )
            return RGB::Black;
        return ((Light*)parent)->getEmission() * abs( tilt );
    }

    double ISphere::intersect( const Ray& ray ) const
    {
        // Algorithm cribbed from smallpt
        // www.kevinbeason.com/smallpt/
        const Vector toCenter = center - ray.getOrigin();
        const double b = toCenter * ray.getDirection();
        const double discriminant = b * b - toCenter * toCenter + radius * radius;
        if ( discriminant < 0 )
            return 0;
        double t;
        const double sqrtDiscriminant = sqrt( discriminant );
        if ( (t = b - sqrtDiscriminant) > EPSILON )
            return t;
        if ( (t = b + sqrtDiscriminant) > EPSILON && !parent->isBackCulled() )
            return t;
        return 0;
    }

    Vector ISphere::getRandomPoint() const
    {
        double a = std::rand() * 2;
        if ( RAND_MAX < a )
            a = RAND_MAX - a;
        double b = std::rand() * 2;
        if ( RAND_MAX < b )
            b = RAND_MAX - b;
        double c = std::rand() * 2;
        if ( RAND_MAX < c )
            c = RAND_MAX - c;
        const Vector randomDirection = (Vector(a, b, c) - center).normalize();
        return center + randomDirection * radius;
    }

    double IPlane::intersect( const Ray& ray ) const
    {
        // Common ray/plane intersection algorithm, see Ogre for example
        // github.com/ehsan/ogre/blob/master/OgreMain/src/OgreMath.cpp
        double denominator = normal * ray.getDirection();
        if ( equal(denominator, 0) )
            return 0;
        else {
            double nominator = offset - normal * ray.getOrigin();
            if ( parent->isBackCulled() && EPSILON < nominator )
                return 0;
            double t = nominator / denominator;
            if ( EPSILON < t )
                return t;
            else
                return 0;
        }
    }

    Vector IPlane::getRandomPoint() const
    {
        const Vector origin = normal * offset;
        const Vector unitX = Vector( origin.y, -origin.x, origin.z ).normalize();
        const Vector unitY = origin.cross( unitX ).normalize();
        const double x = std::rand();
        const double y = std::rand();
        return origin + unitX * x + unitY * y;
    }

    double ITriangle::intersect( const Ray& ray ) const
    {
        // Möller-Trumbore algorithm
        // en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
        // www.scratchapixel.com/old/lessons/3d-basic-lessons/lesson-9-ray-triangle-intersection/m-ller-trumbore-algorithm/
        const Vector edge1 = points[1] - points[0];
        const Vector edge2 = points[2] - points[0];
        const Vector P = ray.getDirection().cross( edge2 );
        const double determinant = edge1 * P;
        if ( parent->isBackCulled() )
        {
            if ( determinant < EPSILON )
                return 0;
        }
        else
        {
            if ( equal(determinant, 0) )
                return 0;
        }
        const Vector T = ray.getOrigin() - points[0];
        double u = T * P / determinant;
        if ( u < 0 || 1 < u )
            return 0;
        const Vector Q = T.cross( edge1 );
        double v = ray.getDirection() * Q / determinant;
        if ( v < 0 || 1 < u + v )
            return 0;
        double t = edge2 * Q / determinant;
        if ( t < EPSILON )
            return 0;
        else
            return t;
    }

    Vector ITriangle::getRandomPoint() const
    {
        // Formula found here:
        // www.cs.princeton.edu/~funk/tog02.pdf
        const double r1 = (double)std::rand() / RAND_MAX;
        const double r2 = (double)std::rand() / RAND_MAX;
        return points[0] * (1 - sqrt(r1)) + points[1] * sqrt(r1) * (1 - r2) + points[2] * sqrt(r1) * r2;
    }

    // Computes the sum of the emissions that reach surfacePoint directly by explicit sampling
    Triplet Scene::getDirectLight( const Vector& surfacePoint, const Vector& surfaceNormal ) const
    {
        Triplet directLightTotal;
        for ( LightIt light = lightsBegin(); light != lightsEnd(); light++ )
        {
            Triplet directLight; // How much of the light intensity from this lightsource actually strikes the point
            for ( int i = 0; i < SHADOWRAYS; ++i )
            {
                const LightPart* lightPart = (*light)->getRandomPart();
                const Vector lightPoint = lightPart->getRandomPoint();
                const Vector toLightPoint = (lightPoint - surfacePoint).normalized();
                if ( surfaceNormal * toLightPoint < 0 )
                    continue;
                Triplet emission = lightPart->getEmission( -toLightPoint );
                if ( RGB::Black == emission )
                    continue;
                const Ray    shadowRay( this, surfacePoint + toLightPoint * EPSILON, toLightPoint, RGB::Black, 1, INF );
                const double distance = (lightPoint - surfacePoint).length() - EPSILON;
                double t = INF;
                bool occluded = false;
                // Lights are non-occluding. Check Things only
                for ( ThingIt thing = thingsBegin(); thing != thingsEnd(); thing++ )
                    if ( (*thing)->isBackground() == false )
                        for ( ThingPartIt part = (*thing)->partsBegin(); part != (*thing)->partsEnd(); part++ )
                            if ( (t = (*part)->intersect(shadowRay)) && t < distance )
                            {
                                occluded = true;
                                break;
                            }
                if ( !occluded && (*light)->isBackground() )
                    for ( ThingIt thing = thingsBegin(); thing != thingsEnd(); thing++ )
                        if ( (*thing)->isBackground() == true )
                            for ( ThingPartIt part = (*thing)->partsBegin(); part != (*thing)->partsEnd(); part++ )
                                if ( (t = (*part)->intersect(shadowRay)) && t < distance )
                                {
                                    occluded = true;
                                    break;
                                }
                if ( !occluded )
                {
                    directLight += emission * (surfaceNormal * toLightPoint) * // As per the Phong model
                                   UNITDIST * UNITDIST / (distance * distance);
                }
            }
            directLight /= SHADOWRAYS;
            directLightTotal += directLight;
        }
        return directLightTotal;
    }

}

