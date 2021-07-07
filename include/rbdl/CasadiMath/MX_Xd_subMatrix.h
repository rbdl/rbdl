/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef MX_XD_SUBMATRIX_H
#define MX_XD_SUBMATRIX_H

#include <vector>
#include <string>
#include <memory>

#include <casadi.hpp>

namespace RBDLCasadiMath {

class MX_Xd_SubMatrix : public casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>{
public:
    MX_Xd_SubMatrix(casadi::MX& mat, const casadi::Slice& i, const casadi::Slice& j) :
        casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>(mat, i, j)
    {

    }
    MX_Xd_SubMatrix(MX_Xd_SubMatrix& mat, const casadi::Slice& i, const casadi::Slice& j) :
        casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>(mat, i, j)
    {

    }
    MX_Xd_SubMatrix(const casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>& me) :
        casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>(me)
    {

    }

    virtual ~MX_Xd_SubMatrix(){

    }

    MX_Xd_scalar norm() const {
        return casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::norm_2(*this);
    }

    void normalize() {
        *this /= norm();
    }

    void operator=(const casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>& submat){
        this->casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator=(submat);
    }
    void operator=(const casadi::MX& mat){
        this->casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator =(mat);
    }
    void operator=(const MX_Xd_scalar& mat){
        this->casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator =(mat);
    }
    void operator=(double mat){
        this->casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator =(mat);
    }
};

}

/* MX_XD_SUBMATRIX_H */
#endif

