find_path(GMATRIX_INCLUDE_DIR NAMES gmatrix.h HINTS /usr/local/include/gmatrix)
find_path(GMATRIX_LINALG_INCLUDE_DIR NAMES gmatrix_linalg.h HINTS /usr/local/include/gmatrix)
find_library(GMATRIX_LIB NAMES gmatrix HINTS /usr/local/lib)
find_library(GMATRIX_LINALG_LIB NAMES gmatrix_linalg HINTS /usr/local/lib)

set(GMATRIX_INCLUDE_DIRS ${GMATRIX_INCLUDE_DIR} ${GMATRIX_LINALG_INCLUDE_DIR})
set(GMATRIX_LIBRARIES ${GMATRIX_LIB} ${GMATRIX_LINALG_LIB})
message(${GMATRIX_LIBRARIES})