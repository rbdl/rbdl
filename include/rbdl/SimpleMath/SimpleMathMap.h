#ifndef SIMPLEMATHMAP_H
#define SIMPLEMATHMAP_H

#include "compileassert.h"

namespace SimpleMath {

/** \brief \brief Wraps a varying size matrix type around existing data
 *
 * \warning If you create a matrix using the map function and then assign
 *  a bigger matrix you invalidate the original memory!
 *
 */
template < typename MatrixType >
MatrixType Map (typename MatrixType::value_type *data, unsigned int rows, unsigned int cols) {
	return MatrixType (rows, cols, data);
}

}

// SIMPLEMATHMAP_H
#endif
