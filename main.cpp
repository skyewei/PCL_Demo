
#include "voxelgrid.h"
#include "pass.h"
#include "statistical_removal.h"
#include "project_inliers.h"
#include "extract_indices.h"
#include "remove_outliers.h"


int main(int argc, char** argv) {

    // processPassthroughFilter();
    // processVoxelGrid();
    // process_statistical_removal();
    // process_project_inliers();
    // process_extract_indices();
    
    process_remove_outliers(argc, argv);

    return 0;
}
