/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef interpolation_h
#define interpolation_h

#include <cml/mathlib/matrix_rotation.h>

/* Interpolation functions.
 *
 * @todo: This code works, but it needs a lot of cleanup.
 */

namespace cml {

struct function_expects_args_of_same_type_error;

namespace detail {

//////////////////////////////////////////////////////////////////////////////
// Helper struct to promote vectors, quaternions, and matrices
//////////////////////////////////////////////////////////////////////////////

template< class T1, class T2, class ResultT > struct TypePromote;

template< class T >
struct TypePromote< T,T,et::scalar_result_tag > {
    typedef T temporary_type;
};

template< class T1, class T2 >
struct TypePromote< T1,T2,et::scalar_result_tag > {
    typedef et::ExprTraits<T1> traits_1;
    typedef et::ExprTraits<T2> traits_2;
    typedef typename traits_1::result_tag result_type_1;
    typedef typename traits_2::result_tag result_type_2;
    
    /* Check that results are of the same type */
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_2>::is_true),
        function_expects_args_of_same_type_error);

    typedef typename et::ScalarPromote<T1,T2>::type temporary_type;
};

template< class T1, class T2 >
struct TypePromote< T1,T2,et::vector_result_tag > {
    typedef et::ExprTraits<T1> traits_1;
    typedef et::ExprTraits<T2> traits_2;
    typedef typename traits_1::result_tag result_type_1;
    typedef typename traits_2::result_tag result_type_2;
    
    /* Check that results are of the same type */
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_2>::is_true),
        function_expects_args_of_same_type_error);

    /* @todo: This should be VectorPromote<> for symmetry with the other
     * type promotions.
     */
    typedef typename CrossPromote<T1,T2>::promoted_vector temporary_type;
};

template< class T1, class T2 >
struct TypePromote< T1,T2,et::matrix_result_tag > {
    typedef et::ExprTraits<T1> traits_1;
    typedef et::ExprTraits<T2> traits_2;
    typedef typename traits_1::result_tag result_type_1;
    typedef typename traits_2::result_tag result_type_2;
    
    /* Check that results are of the same type */
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_2>::is_true),
        function_expects_args_of_same_type_error);

    typedef typename et::MatrixPromote2<T1,T2>::temporary_type temporary_type;
};

template< class T1, class T2 >
struct TypePromote< T1,T2,et::quaternion_result_tag > {
    typedef et::ExprTraits<T1> traits_1;
    typedef et::ExprTraits<T2> traits_2;
    typedef typename traits_1::result_tag result_type_1;
    typedef typename traits_2::result_tag result_type_2;
    
    /* Check that results are of the same type */
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_2>::is_true),
        function_expects_args_of_same_type_error);

    typedef typename et::QuaternionPromote2<T1,T2>::temporary_type
        temporary_type;
};

template< class T1, class T2, class T3, class ResultT > struct TypePromote3;

template< class T1, class T2, class T3 >
struct TypePromote3< T1,T2,T3,et::matrix_result_tag > {
    typedef et::ExprTraits<T1> traits_1;
    typedef et::ExprTraits<T2> traits_2;
    typedef et::ExprTraits<T3> traits_3;
    typedef typename traits_1::result_tag result_type_1;
    typedef typename traits_2::result_tag result_type_2;
    typedef typename traits_3::result_tag result_type_3;
    
    /* Check that results are of the same type */
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_2>::is_true),
        function_expects_args_of_same_type_error);
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_3>::is_true),
        function_expects_args_of_same_type_error);

    typedef typename et::MatrixPromote3<T1,T2,T3>::temporary_type
        temporary_type;
    typedef typename temporary_type::value_type value_type;
};

template< class T1, class T2, class T3 >
struct TypePromote3< T1,T2,T3,et::quaternion_result_tag > {
    typedef et::ExprTraits<T1> traits_1;
    typedef et::ExprTraits<T2> traits_2;
    typedef et::ExprTraits<T3> traits_3;
    typedef typename traits_1::result_tag result_type_1;
    typedef typename traits_2::result_tag result_type_2;
    typedef typename traits_3::result_tag result_type_3;
    
    /* Check that results are of the same type */
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_2>::is_true),
        function_expects_args_of_same_type_error);
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_3>::is_true),
        function_expects_args_of_same_type_error);

    typedef typename et::QuaternionPromote3<T1,T2,T3>::temporary_type
        temporary_type;
    typedef typename temporary_type::value_type value_type;
};

template <
    class T1, class T2, class T3, class T4, class ResultT
> struct TypePromote4;

template< class T1, class T2, class T3, class T4 >
struct TypePromote4< T1,T2,T3,T4,et::matrix_result_tag > {
    typedef et::ExprTraits<T1> traits_1;
    typedef et::ExprTraits<T2> traits_2;
    typedef et::ExprTraits<T3> traits_3;
    typedef et::ExprTraits<T4> traits_4;
    typedef typename traits_1::result_tag result_type_1;
    typedef typename traits_2::result_tag result_type_2;
    typedef typename traits_3::result_tag result_type_3;
    typedef typename traits_4::result_tag result_type_4;
    
    /* Check that results are of the same type */
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_2>::is_true),
        function_expects_args_of_same_type_error);
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_3>::is_true),
        function_expects_args_of_same_type_error);
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_4>::is_true),
        function_expects_args_of_same_type_error);

    typedef typename et::MatrixPromote4<T1,T2,T3,T4>::temporary_type
        temporary_type;
    typedef typename temporary_type::value_type value_type;
};

template< class T1, class T2, class T3, class T4 >
struct TypePromote4< T1,T2,T3,T4,et::quaternion_result_tag > {
    typedef et::ExprTraits<T1> traits_1;
    typedef et::ExprTraits<T2> traits_2;
    typedef et::ExprTraits<T3> traits_3;
    typedef et::ExprTraits<T4> traits_4;
    typedef typename traits_1::result_tag result_type_1;
    typedef typename traits_2::result_tag result_type_2;
    typedef typename traits_3::result_tag result_type_3;
    typedef typename traits_4::result_tag result_type_4;
    
    /* Check that results are of the same type */
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_2>::is_true),
        function_expects_args_of_same_type_error);
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_3>::is_true),
        function_expects_args_of_same_type_error);
    CML_STATIC_REQUIRE_M(
        (same_type<result_type_1, result_type_4>::is_true),
        function_expects_args_of_same_type_error);

    typedef typename et::QuaternionPromote4<T1,T2,T3,T4>::temporary_type
        temporary_type;
    typedef typename temporary_type::value_type value_type;
};

//////////////////////////////////////////////////////////////////////////////
// Helper functions to resize a vector, quaternion or matrix
//////////////////////////////////////////////////////////////////////////////

// Should be able to catch all no-ops with a generic function template...

template < class T1, class T2, class SizeTag > void
InterpResize(T1& t1, const T2& t2, SizeTag) {}

// Catch vector and matrix resizes...

template< typename E, class A, class VecT > void
InterpResize(vector<E,A>& v, const VecT& target, dynamic_size_tag) {
    v.resize(target.size());
}

template< typename E, class A, class B, class L, class MatT > void
InterpResize(matrix<E,A,B,L>& m, const MatT& target, dynamic_size_tag) {
    m.resize(target.rows(),target.cols());
}

//////////////////////////////////////////////////////////////////////////////
// Construction of 'intermediate' quaternions and matrices for use with squad
//////////////////////////////////////////////////////////////////////////////

#if 0
template < class QuatT_1, class QuatT_2 >
typename et::QuaternionPromote2<QuatT_1,QuatT_2>::temporary_type
concatenate_quaternions(
    const QuatT_1& q1,
    const QuatT_2& q2,
    positive_cross)
{
    return q2 * q1;
}

template < class QuatT_1, class QuatT_2 >
typename et::QuaternionPromote2<QuatT_1,QuatT_2>::temporary_type
concatenate_quaternions(
    const QuatT_1& q1,
    const QuatT_2& q2,
    negative_cross)
{
    return q1 * q2;
}

template< class T1, class T2, class T3, class SizeT >
typename detail::TypePromote3<
    T1,T2,T3,typename et::ExprTraits<T1>::result_tag
>::temporary_type
squad_intermediate(
    const T1& t1,
    const T2& t2,
    const T3& t3,
    typename detail::TypePromote3<
        T1, T2, T3, typename et::ExprTraits<T1>::result_tag
    >::value_type tolerance,
    et::quaternion_result_tag,
    SizeT)
{
    typedef et::ExprTraits<T1> traits_1;
    typedef typename traits_1::result_tag result_type_1;

    typedef typename detail::TypePromote3<T1,T2,T3,result_type_1>::temporary_type
        temporary_type;
    typedef typename temporary_type::value_type value_type;
    typedef typename temporary_type::cross_type cross_type;
    typedef et::ExprTraits<temporary_type> result_traits;
    typedef typename result_traits::size_tag size_tag;
    
    /**
     * NOTE: It seems that the equation for computing an intermediate
     * quaternion produces the same results regardless of whether 'standard'
     * or 'reverse' multiplication order is used (I haven't proved this -
     * I've just observed it). Still, just to be sure I've used a pair of
     * helper functions to ensure that the quaternions are multiplied in the
     * right order.
     */

    temporary_type result;
    detail::InterpResize(result, t1, size_tag());

    temporary_type t2_inverse = conjugate(t2);
    temporary_type temp1 = concatenate_quaternions(t1, t2_inverse, cross_type());
    temporary_type temp2 = concatenate_quaternions(t3, t2_inverse, cross_type());
    result = concatenate_quaternions(
        exp(-(log(temp1) + log(temp2)) * value_type(.25)), t2, cross_type());
    return result;
}

/**
 * NOTE: Construction of intermediate rotation matrices for use with squad
 * is currently implemented in terms of quaternions. This is pretty
 * inefficient (especially so in the 2-d case, which involves jumping through
 * a lot of hoops to get to 3-d and back), and is inelegant as well.
 *
 * I imagine this could be streamlined to work directly with the matrices, but
 * I'd need to dig a bit first (figure out the matrix equivalents of
 * quaternion exp() and log(), figure out what shortcuts can be taken in
 * 2-d, etc.), so for now it'll just have to remain as-is.
 *
 * In future versions of the CML, it might also be worth reconsidering
 * whether it's wise to support slerp and squad for matrices. Although it
 * can be done, it's not efficient, and may give the user a false sense of
 * security with respect to the efficiency of the underlying operations.
 */

template< class MatT_1, class MatT_2, class MatT_3, size_t N >
struct squad_intermediate_f;

template< class MatT_1, class MatT_2, class MatT_3 >
struct squad_intermediate_f<MatT_1,MatT_2,MatT_3,3>
{
    template< typename Real >
    typename et::MatrixPromote3< MatT_1,MatT_2,MatT_3 >::temporary_type
    operator()(
        const MatT_1& m1,
        const MatT_2& m2,
        const MatT_3& m3,
        Real tolerance)
    {
        typedef typename et::MatrixPromote3<
            MatT_1,MatT_2,MatT_3 >::temporary_type temporary_type;
        typedef typename temporary_type::value_type value_type;
        typedef quaternion< value_type > quaternion_type;

        quaternion_type q1, q2, q3;
        quaternion_rotation_matrix(q1, m1);
        quaternion_rotation_matrix(q2, m2);
        quaternion_rotation_matrix(q3, m3);

        quaternion_type q4 = squad_intermediate(q1, q2, q3, tolerance);

        temporary_type m;
        et::detail::Resize(m,3,3);
        
        matrix_rotation_quaternion(m, q4);
        
        return m;
    }
};

template< class MatT_1, class MatT_2, class MatT_3 >
struct squad_intermediate_f<MatT_1,MatT_2,MatT_3,2>
{
    template< typename Real >
    typename et::MatrixPromote3< MatT_1,MatT_2,MatT_3 >::temporary_type
    operator()(
        const MatT_1& m1,
        const MatT_2& m2,
        const MatT_3& m3,
        Real tolerance)
    {
        typedef typename et::MatrixPromote3<
            MatT_1,MatT_2,MatT_3 >::temporary_type temporary_type;
        typedef typename temporary_type::value_type value_type;
        typedef quaternion< value_type > quaternion_type;
        typedef vector< value_type, fixed<3> > vector_type;
        
        value_type angle1 = matrix_to_rotation_2D(m1);
        value_type angle2 = matrix_to_rotation_2D(m2);
        value_type angle3 = matrix_to_rotation_2D(m3);
        vector_type axis(value_type(0), value_type(0), value_type(1));
        
        quaternion_type q1, q2, q3;
        quaternion_rotation_axis_angle(q1, axis, angle1);
        quaternion_rotation_axis_angle(q2, axis, angle2);
        quaternion_rotation_axis_angle(q3, axis, angle3);

        quaternion_type q4 = squad_intermediate(q1, q2, q3, tolerance);
        
        value_type angle;
        quaternion_to_axis_angle(q4, axis, angle);

        temporary_type m;
        et::detail::Resize(m,2,2);
        
        matrix_rotation_2D(m, angle);
        
        return m;
    }
};

template< class MatT_1, class MatT_2, class MatT_3, typename Real >
typename et::MatrixPromote3< MatT_1,MatT_2,MatT_3 >::temporary_type
squad_intermediate(
    const MatT_1& m1,
    const MatT_2& m2,
    const MatT_3& m3,
    Real tolerance,
    et::matrix_result_tag,
    fixed_size_tag)
{
    return squad_intermediate_f<MatT_1,MatT_2,MatT_3,MatT_1::array_rows>()(
        m1,m2,m3,tolerance);
}

template< class MatT_1, class MatT_2, class MatT_3, typename Real >
typename et::MatrixPromote3< MatT_1,MatT_2,MatT_3 >::temporary_type
squad_intermediate(
    const MatT_1& m1,
    const MatT_2& m2,
    const MatT_3& m3,
    Real tolerance,
    et::matrix_result_tag,
    dynamic_size_tag)
{
    typedef typename et::MatrixPromote3<
        MatT_1,MatT_2,MatT_3 >::temporary_type temporary_type;

    temporary_type m;
    et::detail::Resize(m,m1.rows(),m1.cols());
    
    switch (m1.rows()) {
        case 3:
            m = squad_intermediate_f<MatT_1,MatT_2,MatT_3,3>()(m1,m2,m3,tolerance);
            break;
        case 2:
            m = squad_intermediate_f<MatT_1,MatT_2,MatT_3,2>()(m1,m2,m3,tolerance);
            break;
        default:
            throw std::invalid_argument(
                "matrix squad_intermediate_f() expects sizes 3x3 or 2x2");
            break;
    }
    return m;
}
#endif

//////////////////////////////////////////////////////////////////////////////
// Spherical linear interpolation of two vectors of any size
//////////////////////////////////////////////////////////////////////////////

template< class VecT_1, class VecT_2, typename Real, class SizeT >
typename detail::TypePromote<
    VecT_1,VecT_2,typename et::ExprTraits<VecT_1>::result_tag
>::temporary_type
slerp(
    const VecT_1& v1,
    const VecT_2& v2,
    Real t,
    Real tolerance,
    et::vector_result_tag,
    SizeT)
{
    typedef et::ExprTraits<VecT_1> type_traits;
    typedef typename type_traits::result_tag result_type;
    typedef typename
        detail::TypePromote<VecT_1,VecT_2,result_type>::temporary_type
            temporary_type;
    typedef typename temporary_type::value_type value_type;
    typedef et::ExprTraits<temporary_type> result_traits;
    typedef typename result_traits::size_tag size_tag;
            
    temporary_type result;
    detail::InterpResize(result, v1, size_tag());

    value_type omega = acos_safe(dot(v1,v2));
    value_type s = std::sin(omega);
    if (s < tolerance) {
        result = nlerp(v1,v2,t);
    } else {
        result = (value_type(std::sin((value_type(1)-t)*omega))*v1 +
            value_type(std::sin(t*omega))*v2) / s;
    }
    return result;
}

//////////////////////////////////////////////////////////////////////////////
// Spherical linear interpolation of two quaternions
//////////////////////////////////////////////////////////////////////////////

template< class QuatT_1, class QuatT_2, typename Real, class SizeT >
typename detail::TypePromote<
    QuatT_1,QuatT_2,typename et::ExprTraits<QuatT_1>::result_tag
>::temporary_type
slerp(
    const QuatT_1& q1,
    const QuatT_2& q2,
    Real t,
    Real tolerance,
    et::quaternion_result_tag,
    SizeT)
{
    typedef et::ExprTraits<QuatT_1> type_traits;
    typedef typename type_traits::result_tag result_type;
    typedef typename
        detail::TypePromote<QuatT_1,QuatT_2,result_type>::temporary_type
            temporary_type;
    typedef typename temporary_type::value_type value_type;

    temporary_type q3 = q2;
    value_type c = dot(q1,q3);
    if (c < value_type(0)) {
        // Turning this off temporarily to test squad...
        q3 = -q3;
        c = -c;
    }
    
    value_type omega = acos_safe(c);
    value_type s = std::sin(omega);

    return (s < tolerance) ?
        normalize(lerp(q1,q3,t)) :
        (value_type(std::sin((value_type(1) - t) * omega)) * q1+
            value_type(std::sin(t * omega)) * q3) / s;
}

//////////////////////////////////////////////////////////////////////////////
// Helper struct for spherical linear interpolation of 3x3 and 2x2 matrices
//////////////////////////////////////////////////////////////////////////////

template< class MatT_1, class MatT_2, size_t N > struct slerp_f;

template< class MatT_1, class MatT_2 > struct slerp_f<MatT_1,MatT_2,3>
{
    template< typename Real >
    typename detail::TypePromote<
        MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
    >::temporary_type
    operator()(
        const MatT_1& m1,
        const MatT_2& m2,
        Real t,
        Real tolerance)
    {
        typedef typename detail::TypePromote<
            MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
        >::temporary_type temporary_type;

        temporary_type m;
        et::detail::Resize(m,3,3);
        m = matrix_rotation_difference(m1,m2);
        matrix_scale_rotation_angle(m,t,tolerance);
        m = detail::matrix_concat_rotations(m1,m);
        return m;
    }
};

template< class MatT_1, class MatT_2 > struct slerp_f<MatT_1,MatT_2,2>
{
    template< typename Real >
    typename detail::TypePromote<
        MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
    >::temporary_type
    operator()(
        const MatT_1& m1,
        const MatT_2& m2,
        Real t,
        Real tolerance)
    {
        typedef typename detail::TypePromote<
            MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
        >::temporary_type temporary_type;

        temporary_type m;
        et::detail::Resize(m,2,2);
        m = matrix_rotation_difference_2D(m1,m2);
        matrix_scale_rotation_angle_2D(m,t,tolerance);
        m = detail::matrix_concat_rotations_2D(m1,m);
        return m;
    }
};

//////////////////////////////////////////////////////////////////////////////
// Spherical linear interpolation of two matrices of size 3x3 or 2x2
//////////////////////////////////////////////////////////////////////////////

template< class MatT_1, class MatT_2, typename Real >
typename detail::TypePromote<
    MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
>::temporary_type
slerp(
    const MatT_1& m1,
    const MatT_2& m2,
    Real t,
    Real tolerance,
    et::matrix_result_tag,
    fixed_size_tag)
{
    return slerp_f<MatT_1,MatT_2,MatT_1::array_rows>()(m1,m2,t,tolerance);
}

template< class MatT_1, class MatT_2, typename Real >
typename detail::TypePromote<
    MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
>::temporary_type
slerp(
    const MatT_1& m1,
    const MatT_2& m2,
    Real t,
    Real tolerance,
    et::matrix_result_tag,
    dynamic_size_tag)
{
    typedef typename detail::TypePromote<
        MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
    >::temporary_type temporary_type;

    temporary_type m;
    et::detail::Resize(m,m1.rows(),m1.cols());
    
    switch (m1.rows()) {
        case 3:
            m = slerp_f<MatT_1,MatT_2,3>()(m1,m2,t,tolerance);
            break;
        case 2:
            m = slerp_f<MatT_1,MatT_2,2>()(m1,m2,t,tolerance);
            break;
        default:
            throw std::invalid_argument(
                "matrix slerp() expects sizes 3x3 or 2x2");
            break;
    }
    return m;
}

//////////////////////////////////////////////////////////////////////////////
// Normalized linear interpolation of two vectors of any size
//////////////////////////////////////////////////////////////////////////////

template< class VecT_1, class VecT_2, typename Real, class SizeT >
typename detail::TypePromote<
    VecT_1,VecT_2,typename et::ExprTraits<VecT_1>::result_tag
>::temporary_type
nlerp(
    const VecT_1& v1,
    const VecT_2& v2,
    Real t,
    et::vector_result_tag,
    SizeT)
{
    typedef et::ExprTraits<VecT_1> type_traits;
    typedef typename type_traits::result_tag result_type;
    typedef typename
        detail::TypePromote<VecT_1,VecT_2,result_type>::temporary_type
            temporary_type;
    typedef typename temporary_type::value_type value_type;
    typedef et::ExprTraits<temporary_type> result_traits;
    typedef typename result_traits::size_tag size_tag;
            
    temporary_type result;
    detail::InterpResize(result, v1, size_tag());

    result = (value_type(1)-t)*v1+t*v2;
    result.normalize();
    return result;
}

//////////////////////////////////////////////////////////////////////////////
// Normalized linear interpolation of two quaternions
//////////////////////////////////////////////////////////////////////////////

template< class QuatT_1, class QuatT_2, typename Real, class SizeT >
typename detail::TypePromote<
    QuatT_1,QuatT_2,typename et::ExprTraits<QuatT_1>::result_tag
>::temporary_type
nlerp(
    const QuatT_1& q1,
    const QuatT_2& q2,
    Real t,
    et::quaternion_result_tag,
    SizeT)
{
    typedef et::ExprTraits<QuatT_1> type_traits;
    typedef typename type_traits::result_tag result_type;
    typedef typename
        detail::TypePromote<QuatT_1,QuatT_2,result_type>::temporary_type
            temporary_type;
    typedef typename temporary_type::value_type value_type;

    return normalize(lerp(q1, (dot(q1,q2) < value_type(0)) ? -q2 : q2, t));
}

//////////////////////////////////////////////////////////////////////////////
// Helper struct for normalized linear interpolation of 3x3 and 2x2 matrices
//////////////////////////////////////////////////////////////////////////////

template< class MatT_1, class MatT_2, size_t N > struct nlerp_f;

template< class MatT_1, class MatT_2 > struct nlerp_f<MatT_1,MatT_2,3>
{
    template< typename Real >
    typename detail::TypePromote<
        MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
    >::temporary_type
    operator()(
        const MatT_1& m1,
        const MatT_2& m2,
        Real t)
    {
        typedef typename detail::TypePromote<
            MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
        >::temporary_type temporary_type;
        typedef typename temporary_type::value_type value_type;

        temporary_type m;
        et::detail::Resize(m,3,3);
        m = lerp(m1,m2,t);
        matrix_orthogonalize_3x3(m);
        return m;
    }
};

template< class MatT_1, class MatT_2 > struct nlerp_f<MatT_1,MatT_2,2>
{
    template< typename Real >
    typename detail::TypePromote<
        MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
    >::temporary_type
    operator()(
        const MatT_1& m1,
        const MatT_2& m2,
        Real t)
    {
        typedef typename detail::TypePromote<
            MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
        >::temporary_type temporary_type;
        typedef typename temporary_type::value_type value_type;

        temporary_type m;
        et::detail::Resize(m,2,2);
        m = lerp(m1,m2,t);
        matrix_orthogonalize_2x2(m);
        return m;
    }
};

//////////////////////////////////////////////////////////////////////////////
// Normalized linear interpolation of two matrices of size 3x3 or 2x2
//////////////////////////////////////////////////////////////////////////////

template< class MatT_1, class MatT_2, typename Real >
typename detail::TypePromote<
    MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
>::temporary_type
nlerp(
    const MatT_1& m1,
    const MatT_2& m2,
    Real t,
    et::matrix_result_tag,
    fixed_size_tag)
{
    return nlerp_f<MatT_1,MatT_2,MatT_1::array_rows>()(m1,m2,t);
}

template< class MatT_1, class MatT_2, typename Real >
typename detail::TypePromote<
    MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
>::temporary_type
nlerp(
    const MatT_1& m1,
    const MatT_2& m2,
    Real t,
    et::matrix_result_tag,
    dynamic_size_tag)
{
    typedef typename detail::TypePromote<
        MatT_1,MatT_2,typename et::ExprTraits<MatT_1>::result_tag
    >::temporary_type temporary_type;

    temporary_type m;
    et::detail::Resize(m,m1.rows(),m1.cols());
    
    switch (m1.rows()) {
        case 3:
            m = nlerp_f<MatT_1,MatT_2,3>()(m1,m2,t);
            break;
        case 2:
            m = nlerp_f<MatT_1,MatT_2,2>()(m1,m2,t);
            break;
        default:
            throw std::invalid_argument(
                "matrix nlerp() expects sizes 3x3 or 2x2");
            break;
    }
    return m;
}

} // namespace detail

//////////////////////////////////////////////////////////////////////////////
// Construction of 'intermediate' quaternions and matrices for use with squad
//////////////////////////////////////////////////////////////////////////////

/**
 * NOTE: Computation of intermediate rotation matrices for matrix 'squad'
 * doesn't seem to be working correctly. I'm not sure what the problem is
 * (it might have to do with q and -q representing the same rotation), but
 * in any case, I don't have time to get it sorted at the moment.
 *
 * In the meantime, I've just hacked in static assertions that will
 * restrict squad usage to quats. For anyone reading these comments, don't
 * worry: the quaternion verison of squad works just fine. However, you'll
 * just have to live without matrix squad for the time being (which is
 * probably just as well, given that matrix interpolation isn't terribly
 * efficient).
 */

#if 0
template< class T1, class T2, class T3 >
typename detail::TypePromote3<
    T1,T2,T3,typename et::ExprTraits<T1>::result_tag
>::temporary_type
squad_intermediate(
    const T1& t1,
    const T2& t2,
    const T3& t3,
    typename detail::TypePromote3<
        T1, T2, T3, typename et::ExprTraits<T1>::result_tag
    >::value_type tolerance =
    epsilon <
        typename detail::TypePromote3<
            T1, T2, T3, typename et::ExprTraits<T1>::result_tag
        >::value_type
    >::placeholder())
{
    // HACK: See note above...
    detail::CheckQuat(t1);
    detail::CheckQuat(t2);
    detail::CheckQuat(t3);

    typedef et::ExprTraits<T1> traits_1;
    typedef typename traits_1::result_tag result_type_1;

    typedef typename detail::TypePromote3<T1,T2,T3,result_type_1>::temporary_type
        temporary_type;
    typedef et::ExprTraits<temporary_type> result_traits;
    typedef typename result_traits::size_tag size_tag;

    temporary_type result;
    detail::InterpResize(result, t1, size_tag());

    result = detail::squad_intermediate(
        t1,t2,t3,tolerance,result_type_1(),size_tag());
    return result;
}

//////////////////////////////////////////////////////////////////////////////
// Spherical quadrangle interpolation of two quaternions or matrices
//////////////////////////////////////////////////////////////////////////////

/**
 * NOTE: The squad() impelementation is unfinished. I'm leaving the code
 * here (but preprocessor'ed out) for future reference.
 *
 * Currently, it seems that:
 *
 * 1. Computation of intermediate matrices is incorrect.
 * 2. The interpolated orientation sometimes 'jumps' while between nodes.
 *
 * I've observed that removing the 'shortest path' negation from the slerp
 * function eliminates the second problem. Also, in another implementation
 * of squad that I've seen, q1 and q2 are interpolated over the shortest
 * path, while the helper quaternions are not. I've never seen this
 * mentioned as a requirement of squad, but maybe they know something I
 * don't.
 *
 * For anyone who happens to read these comments, all of the other
 * interpolation functions (lerp, nlerp, slerp, etc.) should work fine -
 * it's just squad() that's on hold.
 */

template< class T1, class T2, class T3, class T4, typename Real >
typename detail::TypePromote4<
    T1,T2,T3,T4,typename et::ExprTraits<T1>::result_tag
>::temporary_type
squad(
    const T1& t1,
    const T2& t1_intermediate,
    const T3& t2_intermediate,
    const T4& t2,
    Real t,
    Real tolerance = epsilon<Real>::placeholder())
{
    // HACK: See note above...
    detail::CheckQuat(t1);
    detail::CheckQuat(t1_intermediate);
    detail::CheckQuat(t2_intermediate);
    detail::CheckQuat(t2);

    typedef et::ExprTraits<T1> traits_1;
    typedef typename traits_1::result_tag result_type_1;

    typedef typename detail::TypePromote4<
        T1,T2,T3,T4,result_type_1>::temporary_type temporary_type;
    typedef typename temporary_type::value_type value_type;
    typedef et::ExprTraits<temporary_type> result_traits;
    typedef typename result_traits::size_tag size_tag;

    temporary_type result;
    detail::InterpResize(result, t1, size_tag());
    
    result = slerp(
        slerp(t1, t2, t, tolerance),
        slerp(t1_intermediate, t2_intermediate, t, tolerance),
        value_type(2) * t * (value_type(1) - t),
        tolerance
    );

    return result;
}
#endif

//////////////////////////////////////////////////////////////////////////////
// Spherical linear interpolation of two vectors, quaternions or matrices
//////////////////////////////////////////////////////////////////////////////

template< class T1, class T2, typename Real >
typename detail::TypePromote<
    T1,T2,typename et::ExprTraits<T1>::result_tag
>::temporary_type
slerp(
    const T1& t1,
    const T2& t2,
    Real t,
    Real tolerance = epsilon<Real>::placeholder())
{
    typedef et::ExprTraits<T1> traits_1;
    typedef typename traits_1::result_tag result_type_1;

    typedef typename detail::TypePromote<T1,T2,result_type_1>::temporary_type
        temporary_type;
    typedef et::ExprTraits<temporary_type> result_traits;
    typedef typename result_traits::size_tag size_tag;

    temporary_type result;
    detail::InterpResize(result, t1, size_tag());

    result = detail::slerp(t1,t2,t,tolerance,result_type_1(),size_tag());
    return result;
}

//////////////////////////////////////////////////////////////////////////////
// Normalized linear interpolation of two vectors, quaternions or matrices
//////////////////////////////////////////////////////////////////////////////

template< class T1, class T2, typename Real >
typename detail::TypePromote<
    T1,T2,typename et::ExprTraits<T1>::result_tag
>::temporary_type
nlerp(const T1& t1, const T2& t2, Real t)
{
    typedef et::ExprTraits<T1> traits_1;
    typedef typename traits_1::result_tag result_type_1;

    typedef typename detail::TypePromote<T1,T2,result_type_1>::temporary_type
        temporary_type;
    typedef et::ExprTraits<temporary_type> result_traits;
    typedef typename result_traits::size_tag size_tag;

    temporary_type result;
    detail::InterpResize(result, t1, size_tag());

    result = detail::nlerp(t1,t2,t,result_type_1(),size_tag());
    return result;
}

//////////////////////////////////////////////////////////////////////////////
// Linear interpolation of two values of any qualified type
//////////////////////////////////////////////////////////////////////////////

/** Linear interpolation of 2 values.
 *
 * @note The data points are assumed to be sampled at u = 0 and u = 1, so
 * for interpolation u must lie between 0 and 1.
 */
template< class T1, class T2, typename Scalar >
typename detail::TypePromote<
    T1,T2,typename et::ExprTraits<T1>::result_tag
>::temporary_type
lerp(const T1& val0, const T2& val1, Scalar u)
{
    typedef
        typename detail::TypePromote<
            T1,T2,typename et::ExprTraits<T1>::result_tag
        >::temporary_type temporary_type;

    typedef et::ExprTraits<temporary_type> result_traits;
    typedef typename result_traits::size_tag size_tag;

    temporary_type result;
    detail::InterpResize(result, val1, size_tag());
    
    result = (Scalar(1) - u) * val0 + u * val1;
    return result;
}

//////////////////////////////////////////////////////////////////////////////
// Bilinear interpolation of four values of any qualified type
//////////////////////////////////////////////////////////////////////////////

template < class T1, class T2, class T3, class T4, typename Scalar >
typename detail::TypePromote<
    typename detail::TypePromote<
        T1,T2,typename et::ExprTraits<T1>::result_tag
    >::temporary_type,
    typename detail::TypePromote<
        T3,T4,typename et::ExprTraits<T3>::result_tag
    >::temporary_type,
    typename et::ExprTraits<T1>::result_tag
>::temporary_type
bilerp(const T1& val00, const T2& val10,
       const T3& val01, const T4& val11,
       Scalar u, Scalar v)
{
    typedef
        typename detail::TypePromote<
            typename detail::TypePromote<
                T1,T2,typename et::ExprTraits<T1>::result_tag
            >::temporary_type,
            typename detail::TypePromote<
                T3,T4,typename et::ExprTraits<T1>::result_tag
            >::temporary_type,
            typename et::ExprTraits<T1>::result_tag
        >::temporary_type temporary_type;

    typedef et::ExprTraits<temporary_type> result_traits;
    typedef typename result_traits::size_tag size_tag;

    temporary_type result;
    detail::InterpResize(result, val00, size_tag());

    Scalar uv = u * v;
    result = (
        (Scalar(1.0) - u - v + uv) * val00 +
                          (u - uv) * val10 +
                          (v - uv) * val01 +
                                uv * val11
    );
    return result;
}

//////////////////////////////////////////////////////////////////////////////
// Trilinear interpolation of eight values of any qualified type
//////////////////////////////////////////////////////////////////////////////

/** Trilinear interpolation of 8 values.
 *
 * @note The data values are assumed to be sampled at the corners of a unit
 * cube, so for interpolation, u, v, and w must lie between 0 and 1.
 */
template < class T1, class T2, class T3, class T4,
           class T5, class T6, class T7, class T8,
           typename Scalar >
typename detail::TypePromote<
    typename detail::TypePromote<
        typename detail::TypePromote<
            T1,T2,typename et::ExprTraits<T1>::result_tag
        >::temporary_type,
        typename detail::TypePromote<
            T3,T4,typename et::ExprTraits<T3>::result_tag
        >::temporary_type,
        typename et::ExprTraits<T1>::result_tag
    >::temporary_type,
    typename detail::TypePromote<
        typename detail::TypePromote<
            T5,T6,typename et::ExprTraits<T5>::result_tag
        >::temporary_type,
        typename detail::TypePromote<
            T7,T8,typename et::ExprTraits<T7>::result_tag
        >::temporary_type,
        typename et::ExprTraits<T1>::result_tag
    >::temporary_type,
    typename et::ExprTraits<T1>::result_tag
>::temporary_type
trilerp(const T1& val000, const T2& val100,
        const T3& val010, const T4& val110,
        const T5& val001, const T6& val101,
        const T7& val011, const T8& val111,
        Scalar u, Scalar v, Scalar w)
{
    typedef
        typename detail::TypePromote<
            typename detail::TypePromote<
                typename detail::TypePromote<
                    T1,T2,typename et::ExprTraits<T1>::result_tag
                >::temporary_type,
                typename detail::TypePromote<
                    T3,T4,typename et::ExprTraits<T1>::result_tag
                >::temporary_type,
                typename et::ExprTraits<T1>::result_tag
            >::temporary_type,
            typename detail::TypePromote<
                typename detail::TypePromote<
                    T5,T6,typename et::ExprTraits<T1>::result_tag
                >::temporary_type,
                typename detail::TypePromote<
                    T7,T8,typename et::ExprTraits<T1>::result_tag
                >::temporary_type,
                typename et::ExprTraits<T1>::result_tag
            >::temporary_type,
            typename et::ExprTraits<T1>::result_tag
        >::temporary_type temporary_type;

    typedef et::ExprTraits<temporary_type> result_traits;
    typedef typename result_traits::size_tag size_tag;

    temporary_type result;
    detail::InterpResize(result, val000, size_tag());

    Scalar uv = u * v;
    Scalar vw = v * w;
    Scalar wu = w * u;
    Scalar uvw = uv * w;
    
    result = (
        (Scalar(1.0) - u - v - w + uv + vw + wu - uvw) * val000 +
                                   (u - uv - wu + uvw) * val100 +
                                   (v - uv - vw + uvw) * val010 +
                                            (uv - uvw) * val110 +
                                   (w - vw - wu + uvw) * val001 +
                                            (wu - uvw) * val101 +
                                            (vw - uvw) * val011 +
                                                   uvw * val111
    );
    return result;
}

} // namespace cml

#endif
