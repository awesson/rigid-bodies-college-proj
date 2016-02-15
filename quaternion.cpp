/**
 * @file quaternion.cpp
 * @brief A quaternion class.
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "quaternion.h"

#include "matrix.h"

namespace gfx {

const Quaternion Quaternion::Zero( 0.0, 0.0, 0.0, 0.0 );

const Quaternion Quaternion::Identity( 1.0, 0.0, 0.0, 0.0 );

static void make_unit( Quaternion& q )
{
    double maginv = 1.0 / sqrt( norm( q ) );
    q.x *= maginv;
    q.y *= maginv;
    q.z *= maginv;
    q.w *= maginv;
}

Quaternion::Quaternion( const Vec3& axis, double radians )
{
    radians *= 0.5;
    Vec3 naxis = axis;
    unitize( naxis );
    double sine = sin( radians );

    w = cos( radians );
    x = sine * naxis[0];
    y = sine * naxis[1];
    z = sine * naxis[2];

    make_unit( *this );
}

Quaternion::Quaternion( const Matrix3& mat )
{
    // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
    // article "Quaternion Calculus and Fast Animation".

    double trace = mat._m[0][0] + mat._m[1][1] + mat._m[2][2];
    double root;

    if ( trace > 0.0 ) {
        root = sqrt( trace + 1.0 );  // 2w
        w = 0.5 * root;
        root = 0.5 / root;  // 1/(4w)
        x = ( mat._m[1][2] - mat._m[2][1] ) * root;
        y = ( mat._m[2][0] - mat._m[0][2] ) * root;
        z = ( mat._m[0][1] - mat._m[1][0] ) * root;
    } else {
        if ( mat._m[0][0] > mat._m[1][1] && mat._m[0][0] > mat._m[2][2] ) {
            double s = 2.0 * sqrt( 1.0 + mat._m[0][0] - mat._m[1][1] - mat._m[2][2]);
            x = 0.25 * s;
            w = (mat._m[1][2] - mat._m[2][1] ) / s;
            y = (mat._m[1][0] + mat._m[0][1] ) / s;
            z = (mat._m[2][0] + mat._m[0][2] ) / s;
        } else if (mat._m[1][1] > mat._m[2][2]) {
            double s = 2.0 * sqrt( 1.0 + mat._m[1][1] - mat._m[0][0] - mat._m[2][2]);
            y = 0.25 * s;
            w = (mat._m[2][0] - mat._m[0][2] ) / s;
            x = (mat._m[1][0] + mat._m[0][1] ) / s;
            z = (mat._m[2][1] + mat._m[1][2] ) / s;
        } else {
            double s = 2.0 * sqrt( 1.0 + mat._m[2][2] - mat._m[0][0] - mat._m[1][1] );
            z = 0.25 * s;
            w = (mat._m[0][1] - mat._m[1][0] ) / s;
            x = (mat._m[2][0] + mat._m[0][2] ) / s;
            y = (mat._m[2][1] + mat._m[1][2] ) / s;
        }
    }
}

Quaternion::Quaternion( const Matrix4& mat )
{
    // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
    // article "Quaternion Calculus and Fast Animation".

    double trace = mat._m[0][0] + mat._m[1][1] + mat._m[2][2];
    double root;

    if ( trace > 0.0 ) {
        // |w| > 1/2, may as well choose w > 1/2
        root = sqrt( trace + 1.0 );  // 2w
        w = 0.5 * root;
        root = 0.5 / root;  // 1/(4w)
        x = ( mat._m[2][1] - mat._m[1][2] ) * root;
        y = ( mat._m[0][2] - mat._m[2][0] ) * root;
        z = ( mat._m[1][0] - mat._m[0][1] ) * root;
    } else {
        // |w| <= 1/2
        static size_t next[3] = { 1, 2, 0 };
        size_t i = 0;
        if ( mat._m[1][1] > mat._m[0][0] )
            i = 1;
        if ( mat._m[2][2] > mat._m[i][i] )
            i = 2;
        size_t j = next[i];
        size_t k = next[j];

        root = sqrt( mat._m[i][i] - mat._m[j][j] - mat._m[k][k] + 1.0 );
        x = 0.5 * root;
        root = 0.5 / root;
        w = ( mat._m[k][j] - mat._m[j][k] ) * root;
        y = ( mat._m[j][i] + mat._m[i][j] ) * root;
        z = ( mat._m[k][i] + mat._m[i][k] ) * root;
    }
}

Quaternion Quaternion::operator*( const Quaternion& rhs ) const
{
    return Quaternion(
       w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
       w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
       w * rhs.y + y * rhs.w + z * rhs.x - x * rhs.z,
       w * rhs.z + z * rhs.w + x * rhs.y - y * rhs.x
    );
}

Vec3 Quaternion::operator*( const Vec3& v ) const
{
    // nVidia SDK implementation
    Vec3 qvec( x, y, z );
    Vec3 uv = cross( qvec, v );
    Vec3 uuv = cross( qvec, uv );
    uv *= ( 2.0 * w );
    uuv *= 2.0;

    return v + uv + uuv;
}

void Quaternion::to_axis_angle( Vec3* axis, double* angle ) const
{
    // The quaternion representing the rotation is
    // q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)
    double norm = x * x + y * y + z * z;
    if ( norm > 0.0 ) {
        *angle = 2.0 * acos( w );
        double inverse_length = 1 / sqrt( norm );
        (*axis)[0] = x * inverse_length;
        (*axis)[1] = y * inverse_length;
        (*axis)[2] = z * inverse_length;
    } else {
        // angle is 0 (mod 2*pi), so any axis will do
        *angle = 0.0;
        *axis = Vec3(1, 0, 0);
    }
}

static void rotate_axes( const Quaternion& quat,
                         double ax[3], double ay[3], double az[3] )
{
    double x2  = 2.0 * quat.x;
    double y2  = 2.0 * quat.y;
    double z2  = 2.0 * quat.z;
    double xw2 = x2 * quat.w;
    double yw2 = y2 * quat.w;
    double zw2 = z2 * quat.w;
    double xx2 = x2 * quat.x;
    double xy2 = y2 * quat.x;
    double xz2 = z2 * quat.x;
    double yy2 = y2 * quat.y;
    double yz2 = z2 * quat.y;
    double zz2 = z2 * quat.z;

    ax[0] = 1.0 - ( yy2 + zz2 );
    ax[1] = xy2 + zw2;
    ax[2] = xz2 - yw2;

    ay[0] = xy2 - zw2;
    ay[1] = 1.0 - ( xx2 + zz2 );
    ay[2] = yz2 + xw2;

    az[0] = xz2 + yw2;
    az[1] = yz2 - xw2;
    az[2] = 1.0 - ( xx2 + yy2 );
}

void Quaternion::to_matrix( Matrix3* matp ) const
{
    Matrix3& mat = *matp;
    rotate_axes( *this, &mat( 0, 0 ), &mat( 1, 0 ), &mat( 2, 0 ) );
}

void Quaternion::to_matrix( Matrix4* matp ) const
{
    Matrix4& mat = *matp;
    rotate_axes( *this, &mat( 0, 0 ), &mat( 1, 0 ), &mat( 2, 0 ) );

    mat( 0, 3 ) = 0;
    mat( 1, 3 ) = 0;
    mat( 2, 3 ) = 0;
    mat( 3, 0 ) = 0;
    mat( 3, 1 ) = 0;
    mat( 3, 2 ) = 0;
    mat( 3, 3 ) = 1;
}

void Quaternion::to_axes( Vec3 axes[3] ) const
{
    rotate_axes( *this, &axes[0][0], &axes[1][0], &axes[2][0] );
}

Quaternion normalize( const Quaternion& q )
{
    Quaternion rv( q );
    make_unit( rv );
    return rv;
}

Quaternion conjugate( const Quaternion& q )
{
    return Quaternion( q.w, -q.x, -q.y, -q.z );
}

std::ostream& operator <<( std::ostream& o, const Quaternion& q )
{
    o << "Quaternion(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
    return o;
}

} /* _462 */

