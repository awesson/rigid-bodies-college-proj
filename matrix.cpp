/**
 * @file matrix.cpp
 * @brief Matrix classes.
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "matrix.h"
#include "quaternion.h"

#include <cstring>

namespace gfx {

const Matrix3 Matrix3::Identity = Matrix3( 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1 );

const Matrix3 Matrix3::Zero = Matrix3( 0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0 );


Matrix3::Matrix3( double r[SIZE] )
{
    memcpy( m, r, sizeof r );
}

Matrix3::Matrix3( double m00, double m10, double m20,
                  double m01, double m11, double m21,
                  double m02, double m12, double m22 )
{
    _m[0][0] = m00;
    _m[1][0] = m10;
    _m[2][0] = m20;
    _m[0][1] = m01;
    _m[1][1] = m11;
    _m[2][1] = m21;
    _m[0][2] = m02;
    _m[1][2] = m12;
    _m[2][2] = m22;
}

Matrix3::Matrix3( const Vec3& u, const Vec3& v, const Vec3& w )
{
    Vec3* arr = (Vec3*)m;
    arr[0] = u;
    arr[1] = v;
    arr[2] = w;
}

Matrix3 Matrix3::operator+( const Matrix3& rhs ) const
{
    Matrix3 rv;
    for ( int i = 0; i < SIZE; i++ )
        rv.m[i] = m[i] + rhs.m[i];
    return rv;
}

Matrix3& Matrix3::operator+=( const Matrix3& rhs )
{
    for ( int i = 0; i < SIZE; i++ )
        m[i] += rhs.m[i];
    return *this;
}

Matrix3 Matrix3::operator-( const Matrix3& rhs ) const
{
    Matrix3 rv;
    for ( int i = 0; i < SIZE; i++ )
        rv.m[i] = m[i] - rhs.m[i];
    return rv;
}

Matrix3& Matrix3::operator-=( const Matrix3& rhs )
{
    for ( int i = 0; i < SIZE; i++ )
        m[i] -= rhs.m[i];
    return *this;
}

Matrix3 Matrix3::operator*( const Matrix3& rhs ) const
{
    Matrix3 product;
    for ( int i = 0; i < DIM; ++i )
        for ( int j = 0; j < DIM; ++j )
            product._m[i][j] =
                _m[0][j] * rhs._m[i][0] + _m[1][j] * rhs._m[i][1] +
                _m[2][j] * rhs._m[i][2];
    return product;
}

Vec3 Matrix3::operator*( const Vec3& v ) const
{
    return Vec3( _m[0][0]*v[0] + _m[1][0]*v[1] + _m[2][0]*v[2],
                    _m[0][1]*v[0] + _m[1][1]*v[1] + _m[2][1]*v[2],
                    _m[0][2]*v[0] + _m[1][2]*v[1] + _m[2][2]*v[2] );
}

Matrix3& Matrix3::operator*=( const Matrix3& rhs )
{
    return *this = operator*( rhs );
}

Matrix3 Matrix3::operator*( double r ) const
{
    Matrix3 rv;
    for ( int i = 0; i < SIZE; i++ )
        rv.m[i] = m[i] * r;
    return rv;
}

Matrix3& Matrix3::operator*=( double r )
{
    for ( int i = 0; i < SIZE; i++ )
        m[i] *= r;
    return *this;
}

Matrix3 Matrix3::operator/( double r ) const
{
    Matrix3 rv;
    double inv = 1 / r;
    for ( int i = 0; i < SIZE; i++ )
        rv.m[i] = m[i] * inv;
    return rv;
}

Matrix3& Matrix3::operator/=( double r )
{
    double inv = 1 / r;
    for ( int i = 0; i < SIZE; i++ )
        m[i] *= inv;
    return *this;
}

Matrix3 Matrix3::operator-() const
{
    Matrix3 rv;
    for ( int i = 0; i < SIZE; i++ )
        rv.m[i] = -m[i];
    return rv;
}

bool Matrix3::operator==( const Matrix3& rhs ) const
{
    return memcmp( m, rhs.m, sizeof m ) == 0;
}

bool Matrix3::operator!=( const Matrix3& rhs ) const
{
    return !operator==( rhs );
}

void transpose( Matrix3* rv, const Matrix3& m )
{
    rv->_m[0][0] = m._m[0][0];
    rv->_m[0][1] = m._m[1][0];
    rv->_m[0][2] = m._m[2][0];
    rv->_m[1][0] = m._m[0][1];
    rv->_m[1][1] = m._m[1][1];
    rv->_m[1][2] = m._m[2][1];
    rv->_m[2][0] = m._m[0][2];
    rv->_m[2][1] = m._m[1][2];
    rv->_m[2][2] = m._m[2][2];
}

void inverse( Matrix3* rv, const Matrix3& m )
{
    rv->_m[0][0] = m._m[1][1] * m._m[2][2] - m._m[1][2] * m._m[2][1];
    rv->_m[0][1] = m._m[0][2] * m._m[2][1] - m._m[0][1] * m._m[2][2];
    rv->_m[0][2] = m._m[0][1] * m._m[1][2] - m._m[0][2] * m._m[1][1];
    rv->_m[1][0] = m._m[1][2] * m._m[2][0] - m._m[1][0] * m._m[2][2];
    rv->_m[1][1] = m._m[0][0] * m._m[2][2] - m._m[0][2] * m._m[2][0];
    rv->_m[1][2] = m._m[0][2] * m._m[1][0] - m._m[0][0] * m._m[1][2];
    rv->_m[2][0] = m._m[1][0] * m._m[2][1] - m._m[1][1] * m._m[2][0];
    rv->_m[2][1] = m._m[0][1] * m._m[2][0] - m._m[0][0] * m._m[2][1];
    rv->_m[2][2] = m._m[0][0] * m._m[1][1] - m._m[0][1] * m._m[1][0];

    double det = m._m[0][0] * rv->_m[0][0] +
                 m._m[0][1] * rv->_m[1][0] +
                 m._m[0][2] * rv->_m[2][0];

    double invdet = 1.0 / det;
    for ( int i = 0; i < Matrix3::SIZE; i++ )
        rv->m[i] *= invdet;
}

const Matrix4 Matrix4::Identity = Matrix4( 1, 0, 0, 0,
                                           0, 1, 0, 0,
                                           0, 0, 1, 0,
                                           0, 0 , 0, 1 );

const Matrix4 Matrix4::Zero = Matrix4( 0, 0, 0, 0,
                                       0, 0, 0, 0,
                                       0, 0, 0, 0,
                                       0, 0, 0, 0 );

Matrix4::Matrix4( double r[SIZE] )
{
    memcpy( m , r, sizeof r );
}

Matrix4::Matrix4( double m00, double m10, double m20, double m30,
                  double m01, double m11, double m21, double m31,
                  double m02, double m12, double m22, double m32,
                  double m03, double m13, double m23, double m33 )
{
    _m[0][0] = m00;
    _m[1][0] = m10;
    _m[2][0] = m20;
    _m[3][0] = m30;
    _m[0][1] = m01;
    _m[1][1] = m11;
    _m[2][1] = m21;
    _m[3][1] = m31;
    _m[0][2] = m02;
    _m[1][2] = m12;
    _m[2][2] = m22;
    _m[3][2] = m32;
    _m[0][3] = m03;
    _m[1][3] = m13;
    _m[2][3] = m23;
    _m[3][3] = m33;
}


Matrix4 Matrix4::operator+( const Matrix4& rhs ) const
{
    Matrix4 rv;
    for ( int i = 0; i < SIZE; i++ )
        rv.m[i] = m[i] + rhs.m[i];
    return rv;
}

Matrix4& Matrix4::operator+=( const Matrix4& rhs )
{
    for ( int i = 0; i < SIZE; i++ )
        m[i] += rhs.m[i];
    return *this;
}

Matrix4 Matrix4::operator-( const Matrix4& rhs ) const
{
    Matrix4 rv;
    for ( int i = 0; i < SIZE; i++ )
        rv.m[i] = m[i] - rhs.m[i];
    return rv;
}

Matrix4& Matrix4::operator-=( const Matrix4& rhs )
{
    for ( int i = 0; i < SIZE; i++ )
        m[i] -= rhs.m[i];
    return *this;
}

Matrix4 Matrix4::operator*( const Matrix4& rhs ) const
{
    Matrix4 product;
    for ( int i = 0; i < DIM; ++i )
        for ( int j = 0; j < DIM; ++j )
            product._m[i][j] =
                _m[0][j] * rhs._m[i][0] + _m[1][j] * rhs._m[i][1] +
                _m[2][j] * rhs._m[i][2] + _m[3][j] * rhs._m[i][3];
    return product;
}

Vec4f Matrix4::operator*( const Vec4f& v ) const
{
    return Vec4f( _m[0][0]*v[0] + _m[1][0]*v[1] + _m[2][0]*v[2] + _m[3][0]*v[3],
                    _m[0][1]*v[0] + _m[1][1]*v[1] + _m[2][1]*v[2] + _m[3][1]*v[3],
                    _m[0][2]*v[0] + _m[1][2]*v[1] + _m[2][2]*v[2] + _m[3][2]*v[3],
                    _m[0][3]*v[0] + _m[1][3]*v[1] + _m[2][3]*v[2] + _m[3][3]*v[3] );
}

Matrix4& Matrix4::operator*=( const Matrix4& rhs )
{
    return *this = operator*( rhs );
}

Matrix4 Matrix4::operator*( double r ) const
{
    Matrix4 rv;
    for ( int i = 0; i < SIZE; i++ )
        rv.m[i] = m[i] * r;
    return rv;
}

Matrix4& Matrix4::operator*=( double r )
{
    for ( int i = 0; i < SIZE; i++ )
        m[i] *= r;
    return *this;
}

Matrix4 Matrix4::operator/( double r ) const
{
    Matrix4 rv;
    double inv = 1 / r;
    for ( int i = 0; i < SIZE; i++ )
        rv.m[i] = m[i] * inv;
    return rv;
}

Matrix4& Matrix4::operator/=( double r )
{
    double inv = 1 / r;
    for ( int i = 0; i < SIZE; i++ )
        m[i] *= inv;
    return *this;
}

Matrix4 Matrix4::operator-() const
{
    Matrix4 rv;
    for ( int i = 0; i < SIZE; i++ )
        rv.m[i] = -m[i];
    return rv;
}

bool Matrix4::operator==( const Matrix4& rhs ) const
{
    return memcmp( m, rhs.m, sizeof m ) == 0;
}

bool Matrix4::operator!=( const Matrix4& rhs ) const
{
    return !operator==( rhs );
}

static void make_translation_matrix( Matrix4* mat, const Vec3& pos )
{
    *mat = Matrix4(
        1.0, 0.0, 0.0, pos[0],
        0.0, 1.0, 0.0, pos[1],
        0.0, 0.0, 1.0, pos[2],
        0.0, 0.0, 0.0, 1.0 );
}

static void make_scaling_matrix( Matrix4* mat, const Vec3& scl )
{
    *mat = Matrix4(
        scl[0], 0.0,   0.0,   0.0,
        0.0,   scl[1], 0.0,   0.0,
        0.0,   0.0,   scl[2], 0.0,
        0.0,   0.0,   0.0,   1.0 );
}


void make_transformation_matrix(
    Matrix4* mat, const Vec3& pos, const Quaternion& ori, const Vec3& scl )
{
    Matrix4 sclmat, orimat;

    ori.to_matrix( &orimat );
    make_scaling_matrix( &sclmat, scl );
    *mat = orimat * sclmat;
    // don't need to actually do the multiplication, can take shortcut
    // since we're multiplying translation by a linear matrix
    mat->m[12] = pos[0];
    mat->m[13] = pos[1];
    mat->m[14] = pos[2];
}

void make_inverse_transformation_matrix(
    Matrix4* mat, const Vec3& pos, const Quaternion& ori, const Vec3& scl )
{
    // assumes orientation is normalized
    Matrix4 sclmat, orimat, trnmat;

    make_scaling_matrix( &sclmat, Vec3( 1.0 / scl[0], 1.0 / scl[1], 1.0 / scl[3] ) );
    conjugate( ori ).to_matrix( &orimat );
    make_translation_matrix( &trnmat, -pos );

    *mat = sclmat * orimat * trnmat;
}

/** algorithm from:  http://www.mathwords.com/c/cofactor.htm **/
void make_normal_matrix( Matrix3* rv, const Matrix4& tmat )
{
    Matrix3 tmp1, tmp2;

    tmp1._m[0][0] = tmat._m[0][0];
    tmp1._m[0][1] = tmat._m[0][1];
    tmp1._m[0][2] = tmat._m[0][2];
    tmp1._m[1][0] = tmat._m[1][0];
    tmp1._m[1][1] = tmat._m[1][1];
    tmp1._m[1][2] = tmat._m[1][2];
    tmp1._m[2][0] = tmat._m[2][0];
    tmp1._m[2][1] = tmat._m[2][1];
    tmp1._m[2][2] = tmat._m[2][2];

    inverse( &tmp2, tmp1 );
    transpose( rv, tmp2 );
}

} /* _462 */

