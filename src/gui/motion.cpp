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

// Different types of Object motions
// Part of Retra, the Reference Tracer

#include "motion.h"

#include <cmath>
#include <cstdlib>

namespace Retra {

    void BrownianMotion::step( double dt ) const
    {
        double x = std::rand() * 2;
        if ( RAND_MAX < x )
            x = RAND_MAX - x;
        double y = std::rand() * 2;
        if ( RAND_MAX < y )
            y = RAND_MAX - y;
        double z = std::rand() * 2;
        if ( RAND_MAX < z )
            z = RAND_MAX - z;
        Vector delta = Vector( x, y, z ).normalized() * scale;

        object->move( delta * dt );
    }

    void LinearMotion::step( double dt ) const
    {
        if ( stop < 0 || distance < stop )
        {
            object->move( delta * dt );
            distance += delta.length() * dt;
        }
    }

    void OrbitingMotion::step( double dt ) const
    {
        object->move( 2.0 * PI * dt / period, axis );
    }

    void OscillatingMotion::step( double dt ) const
    {
        object->move( (end - begin) / PI * sin(phase) * dt );
        phase += 2.0 * PI * dt / period; // TODO: don't let phase overflow.
    }

}

