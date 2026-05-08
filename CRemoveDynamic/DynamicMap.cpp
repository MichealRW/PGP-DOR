#include "DynamicMap.h"

#include <filesystem>

namespace {

std::filesystem::path ResolveOutputRoot(const std::string &requested_path) {
    std::filesystem::path requested_root(requested_path);
    std::error_code error_code;

    std::filesystem::create_directories(requested_root / "result_pgp", error_code);
    if (!error_code) {
        return requested_root;
    }

    std::filesystem::path fallback_root =
        std::filesystem::current_path() / "outputs" / requested_root.filename();
    std::filesystem::create_directories(fallback_root / "result_pgp");
    return fallback_root;
}

}  // namespace


CGlobalGridCellV2::CGlobalGridCellV2(/*double reference_height_*/)
{
    ob_min_height = - INVALID_HEIGHT;
    terrain_height = INVALID_HEIGHT;
    max_overlap = 0;
    //reference_height = reference_height_;

    observation_this = false;
    terrain_in = false;

    curr_state = false;
    last_state = false;
    p_value = false;
    //no_obcheck = true;
    conti_n = 1;
    n = 0;
    num_pt = 0;
}

CGlobalGridCellV2::~CGlobalGridCellV2()
{

}

void CGlobalGridCellV2::AddObservation(float ob_global_point_height, int current_overlap, int num_clouds_in)
{
    //observation_this = true;
    p_value = true;
    if(current_overlap>max_overlap){
        max_overlap = current_overlap;
    }

    if(current_overlap >= MAX_SCORE/2){
        if(ob_min_height > ob_global_point_height /* - reference_height*/){
            ob_min_height = ob_global_point_height;
        }
    }
        
}

void CGlobalGridCellV2::TerrainObservation()
{
    observation_this = true;
}

void CGlobalGridCellV2::OverCheck()
{
    if(!observation_this)
        return;

    if(!p_value)
        max_overlap = MAX_SCORE/4;

    if(max_overlap >= MAX_SCORE/3){
        curr_state = true;
    }else{
        curr_state = false;
    }

    if(last_state == curr_state){
        conti_n++;
    }else{
        conti_n = 1;
    }

    int zhen_fu, mean_score;
    if(curr_state){
        mean_score = max_overlap;
        zhen_fu = 1;
    }else{
        mean_score = MAX_SCORE - max_overlap;
        zhen_fu = -1;
    }

    n += zhen_fu * pow(mean_score, conti_n);

    if(n <= 0)
        ob_min_height = - INVALID_HEIGHT;

    last_state = curr_state;
}

void CGlobalGridCellV2::OverCheckV2()
{
    if(!observation_this)
        return;

    // if(!p_value)
    //     max_overlap = MAX_SCORE/5;

    if(max_overlap >= MAX_SCORE/2){
        curr_state = true;
    }else{
        curr_state = false;
    }

    if(last_state == curr_state){
        conti_n++;
    }else{
        conti_n = 1;
    }

    int mean_score;
    if(curr_state){
        mean_score = max_overlap;
    }else{
        mean_score = max_overlap - MAX_SCORE;
    }

    n += mean_score * exp(conti_n);

    if(n <= 0)
        ob_min_height = - INVALID_HEIGHT;

    last_state = curr_state;
}

void CGlobalGridCellV2::SetTerrainHeight(double terrain_height_)
{
    //terrain_height = (terrain_height_ + terrain_height)/2;
    //terrain_height = terrain_height_;

    if(terrain_in){
        terrain_height = (terrain_height_ + terrain_height)/2;
    }else{
        terrain_height = terrain_height_;
        terrain_in = true;
    }
}

void CGlobalGridCellV2::ReBool()
{
    max_overlap = 0;
    observation_this = false;
    p_value = false;
}

////////////////////////////////////////////////////////////////////////////////////////

CGlobalGridCell::CGlobalGridCell(/*double reference_height_*/)
{
    ob_min_height = - INVALID_HEIGHT;
    terrain_height = INVALID_HEIGHT;
    //reference_height = reference_height_;
    p_value = true;
    observation_this = false;
    terrain_in = false;
    //no_obcheck = true;
    n = 0;
}

CGlobalGridCell::~CGlobalGridCell()
{

}

void CGlobalGridCell::AddObservation(float ob_global_point_height, int current_overlap, int num_clouds_in)
{
    observation_this = true;
    if(current_overlap >= MAX_SCORE/2){
        if(ob_min_height > ob_global_point_height /* - reference_height*/){
            ob_min_height = ob_global_point_height;
        }
        if(p_value){
            n += 5;
            p_value = false;
        }
    }
}

void CGlobalGridCell::TerrainObservation()
{
    observation_this = true;
}

////////////////////////////////////////////////////////////////////////////////////

void CGlobalGridCell::NoObCheck()
{
    // && observation_this
    if(p_value && n > 0 && observation_this){
        n -= 10;
    }

    if(n <= 0){
        ob_min_height = - INVALID_HEIGHT;
        //n = 0;
    }
}

void CGlobalGridCell::NoObCheckV2()
{
    if(p_value && n > 0 && observation_this){
        n -= 6;
    }

    if(n <= 0){
        ob_min_height = - INVALID_HEIGHT;
        n = 0;
    }
}

void CGlobalGridCell::SetTerrainHeight(double terrain_height_)
{
    //terrain_height = (terrain_height_ + terrain_height)/2;
    //terrain_height = terrain_height_;

    if(terrain_in){
        terrain_height = (terrain_height_ + terrain_height)/2;
    }else{
        terrain_height = terrain_height_;
        terrain_in = true;
    }
}

void CGlobalGridCell::ReBool()
{
    p_value = true;
    observation_this = false;
}

/////////////////////////////////////////////////////////////////////////////////

DynamicMap::DynamicMap()
{
    //ikdtree_map.set_downsample_param(0.3);
    vis == NULL;
    points_in = 0;
    dy_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ndy_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
    dy_points_tmp.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    ndy_points_tmp.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    Big_x_bei = BIG_BEISHU;
    Big_y_bei = BIG_BEISHU;
    no_storage_in = true;
}

DynamicMap::~DynamicMap()
{
    
}

void DynamicMap::SetStartFrame(int start_frame_)
{
    start_frame = start_frame_;
}

void DynamicMap::SetMapResolution(double map_resolution_)
{
    map_resolution = map_resolution_;
}

void DynamicMap::SetReferenceHeight(double reference_height_)
{
    reference_height = reference_height_;
}

void DynamicMap::SetROIRange(float roi_range_)
{
    roi_range = roi_range_;
}

void DynamicMap::SetMapSize(int map_rows_, int map_cols_)
{
    map_cols = map_cols_;
    map_rows = map_rows_;
}

void DynamicMap::SetAccumedTerrainMapper(CAccumedTerrainMapper *accumed_terrain_mapper_)
{
    accumed_terrain_mapper = accumed_terrain_mapper_;
}

void DynamicMap::SetInstantTerrainMapper(CInstantTerrainMapper *instant_terrain_mapper_)
{
    instant_terrain_mapper = instant_terrain_mapper_;
}

void DynamicMap::SetTwlRwl(Eigen::Vector3d T_w_l, Eigen::Matrix3d R_w_l)
{
    t_w_l(0) = T_w_l(0);
    t_w_l(1) = T_w_l(1);
    t_w_l(2) = T_w_l(2);
    r_w_l(0) = R_w_l(0);
    r_w_l(1) = R_w_l(1);
    r_w_l(2) = R_w_l(2);
    //current_center_x = t_w_l(0);
    //current_center_y = t_w_l(1);
    current_residual_x = std::floor(t_w_l(0) / map_resolution) * map_resolution - t_w_l(0);
    current_residual_y = std::floor(t_w_l(1) / map_resolution) * map_resolution - t_w_l(1);
    if(points_in == 0 && no_storage_in){
        inital_residual_x = std::floor(t_w_l(0) / map_resolution) * map_resolution - t_w_l(0);
        inital_residual_y = std::floor(t_w_l(1) / map_resolution) * map_resolution - t_w_l(1);
        inital_center_x = t_w_l(0);
        inital_center_y = t_w_l(1);
    }
    OverBoundaryCheck();
}

void DynamicMap::OverBoundaryCheck()
{
    double dist_x = t_w_l(0) - inital_center_x;
    double dist_y = t_w_l(1) - inital_center_y;
    double x_max_b = map_resolution * map_cols * (Big_x_bei - 1) / 2;
    double y_max_b = map_resolution * map_rows * (Big_y_bei - 1) / 2;
    if(dist_x >= x_max_b){
        ChangeBigMat(1);
        inital_center_x += map_resolution * map_cols;
    }else if(dist_x <= -x_max_b){
        ChangeBigMat(2);
        inital_center_x -= map_resolution * map_cols;
    }else if(dist_y >= y_max_b){
        ChangeBigMat(3);
        inital_center_y += map_resolution * map_rows;
    }else if(dist_y <= -y_max_b){
        ChangeBigMat(4);
        inital_center_y -= map_resolution * map_rows;
    }else{
        return;
    }
    LoadMat();
}

void DynamicMap::ChangeBigMat(int flag)
{

    int new_big_x = Big_x_bei, new_big_y = Big_y_bei;
    int c_offset = 0;
    int r_offset = 0;

    switch (flag)
    {
    case 1:
    case 2:
        new_big_x = Big_x_bei + 2;
        c_offset = flag - 1;
        c_offset = c_offset * 2 * map_cols;
        break;
    case 3:
    case 4:
        new_big_y = Big_y_bei + 2;
        r_offset = 4 - flag;
        r_offset = r_offset * 2 * map_rows;
        break;
    default:
        break;
    }
    cv::Mat new_big_ndy_map_mat = cv::Mat::zeros(map_rows * new_big_y, map_cols * new_big_x, CV_64FC1) - INVALID_HEIGHT;
    for(int r = 0; r < map_rows * Big_y_bei; r++){
        for(int c = 0; c < map_cols * Big_x_bei; c++){
            new_big_ndy_map_mat.at<double>(r + r_offset,c + c_offset) = big_ndy_map_mat.at<double>(r,c);
        }
    }
    big_ndy_map_mat = new_big_ndy_map_mat.clone();
    Big_x_bei = new_big_x;
    Big_y_bei = new_big_y;
}

void DynamicMap::SetBGKSmoother(BGK_Smoother *bgk_smoother_)
{
    bgk_smoother = bgk_smoother_;
}

void DynamicMap::SetCheckmode(int mode_)
{
    check_mode = mode_;
}

void DynamicMap::Initialize()
{
    //big_global_ob_map = new CExpandableMap<CGlobalGridCell>(map_resolution, map_rows * Big_y_bei, map_cols * Big_y_bei);

    big_ndy_map_mat = cv::Mat::zeros(map_rows * Big_y_bei, map_cols * Big_x_bei, CV_64FC1) - INVALID_HEIGHT;
    big_ndy_term_map_mat = cv::Mat::zeros(map_rows * Big_y_bei, map_cols * Big_x_bei, CV_64FC1);
    ndy_predict_map_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    small_ndy_map_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    predicted_terrain_height_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    
    dy_points_tmp->clear();
    ndy_points_tmp->clear();

    DLoadMat();
}

void DynamicMap::ResetIntermediateVariables()
{
    ndy_predict_map_mat.setTo(INVALID_HEIGHT);
    small_ndy_map_mat.setTo(INVALID_HEIGHT);
}

void DynamicMap::PointsInOC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr oc_pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::octree::OctreePointCloud<pcl::PointXYZ> current_octree(0.3);
    pcl::PointXYZ pt;
    for(size_t i = 0; i < cloud->size(); i++)
    {
        pt.x = cloud->points[i].x + t_w_l(0);
        pt.y = cloud->points[i].y + t_w_l(1);
        pt.z = cloud->points[i].z + t_w_l(2);
        oc_pts->points.push_back(pt);

        // if(cloud->points[i].z > 0.4){
        //     continue;
        // }

        int current_overlap = 0;    //初段检测
        int hole_one = MIN_CLOUDS_IN;
        if(points_in >= MIN_CLOUDS_IN){
            for(auto it = ndy_octree_buffer.begin(); it != ndy_octree_buffer.end(); it++){
            //for(auto it = ndy_octree_buffer.begin() + points_in - MIN_CLOUDS_IN - 1; it != ndy_octree_buffer.end(); it++){
                if((*it).isVoxelOccupiedAtPoint(pt.x, pt.y, pt.z)){
                    current_overlap += hole_one;
                    hole_one--;
                }
            }
        }
        
        int32_t c = map_cols * Big_x_bei / 2 + std::floor((pt.x - inital_center_x - inital_residual_x) / map_resolution);
        int32_t r = map_rows * Big_y_bei / 2 - 1 - std::floor((pt.y - inital_center_y - inital_residual_y) / map_resolution);

        if(current_overlap >= 3 * MIN_CLOUDS_IN){
            // if(r<0 || r>=map_rows*Big_y_bei || c<0 || c>=map_cols*Big_x_bei){
            //     continue;
            // }else{
            //     big_ndy_map_mat.at<double>(r,c) = pt.z - reference_height;
            // }

            if(big_ndy_map_mat.at<double>(r,c) > pt.z - reference_height)
                big_ndy_map_mat.at<double>(r,c) = pt.z - reference_height;
        }
    }

    current_octree.setInputCloud(oc_pts);
    current_octree.addPointsFromInputCloud();
    ndy_octree_buffer.push_back(current_octree);

    if(points_in >= MIN_CLOUDS_IN)
    {
        //cloud->clear();
        points_in++;
        dy_points->clear();
        ndy_points->clear();
        DynamicVoteOC(oc_pts, cloud);
        ndy_octree_buffer.pop_front();
    }else{
        points_in++;
    }

}

void DynamicMap::DynamicVoteOC(pcl::PointCloud<pcl::PointXYZ>::Ptr ob_cld, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    int32_t c_offset = map_cols * Big_x_bei / 2 + std::floor((t_w_l(0) - inital_center_x - inital_residual_x) / map_resolution) - map_cols / 2;
    int32_t r_offset = map_rows * Big_y_bei / 2 - 1 - std::floor((t_w_l(1) - inital_center_y - inital_residual_y) / map_resolution) - map_rows / 2;
    ResetIntermediateVariables();

    for(int r=0; r<map_rows; r++){
        for(int c=0; c<map_cols; c++){
            if(r + r_offset<0 || r + r_offset>=map_rows*Big_y_bei || c + c_offset<0 || c + c_offset>=map_cols*Big_x_bei){
                continue;
            }else{
                if(big_ndy_map_mat.at<double>(r + r_offset,c + c_offset) == -INVALID_HEIGHT)
                    continue;
                
                if(big_ndy_map_mat.at<double>(r + r_offset,c + c_offset) - predicted_terrain_height_mat.at<double>(r,c) < MAX_DYNAMIC){
                    small_ndy_map_mat.at<double>(r,c) = predicted_terrain_height_mat.at<double>(r,c) - DEEP_ABSORT;
                }else{
                    small_ndy_map_mat.at<double>(r,c) = big_ndy_map_mat.at<double>(r + r_offset,c + c_offset) - DEEP_ABSORT; 
                }
            }
        }
    }

    bgk_smoother->Run(small_ndy_map_mat, ndy_predict_map_mat, INVALID_HEIGHT, false);
    //VisualizeMat(ndy_predict_map_mat, "predict_stable_mat", false);
    VisualizeMatWithInvalidValue(ndy_predict_map_mat, INVALID_HEIGHT, "predict_stable_mat", false);

    pcl::PointXYZ pt;
    for(size_t i=0; i< cloud->points.size(); i++){
        float x = ob_cld->points[i].x;pt.x = cloud->points[i].x;
        float y = ob_cld->points[i].y;pt.y = cloud->points[i].y;
        float z = ob_cld->points[i].z;pt.z = cloud->points[i].z;
        
        int32_t c = map_cols * Big_x_bei / 2 + std::floor((x - inital_center_x - inital_residual_x) / map_resolution) - c_offset;
        int32_t r = map_rows * Big_y_bei / 2 - 1 - std::floor((y - inital_center_y - inital_residual_y) / map_resolution) - r_offset;

        if(r<0 || r>=map_rows || c<0 || c>=map_cols){
            continue;
        }else{
            //CAccumedTerrainCellV2 *this_accume_cell = accumed_terrain_mapper->accumed_terrain_map->GetRCLocal(r, c);
            if(ndy_predict_map_mat.at<double>(r,c) == INVALID_HEIGHT || ndy_predict_map_mat.at<double>(r,c) > z - reference_height){
                dy_points->points.push_back(pt);
            }else{
                ndy_points->points.push_back(pt);
            }
            // if(ndy_predict_map_mat.at<double>(r,c) == INVALID_HEIGHT && z - reference_height < this_accume_cell->avg_height + 2){
            //     if(this_accume_cell->avg_height != INVALID_HEIGHT)
            //         dy_points->points.push_back(pt);
            // }else{
            //     ndy_points->points.push_back(pt);
            // }
        }
    }   
}

float DynamicMap::StdMatchOut(std::vector<float> distance)
{
    float std_distance = 0;
    int num_dis = 0;
    for(auto iter = distance.begin(); iter != distance.end(); iter++)
    {
        num_dis++;
        std_distance += (*iter);
    }
    if(num_dis == 0){
        std_distance = 0;
    }else{
        std_distance = std_distance / num_dis;
    }
    return std_distance;
}

void DynamicMap::Addpoint2clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2)
{
    pcl::PointXYZ pt;
    for(size_t i = 0; i < cloud1->size(); i++)
    {
        std::lock_guard<std::mutex> lg(mtx_point);
        pt = cloud1->points[i];
        cloud2->points.push_back(pt);
    }
}

void DynamicMap::LoadMat()
{
    FILE *fptl;
    std::string load_center_path = std::string("./oldmap/big_center.txt");
    fptl = fopen(load_center_path.c_str(), "w");
    fprintf(fptl, "%lf %lf %f %f %f %d %d %f\n", inital_residual_x, inital_residual_y, inital_center_x, inital_center_y, reference_height, Big_x_bei, Big_y_bei);
    fclose(fptl);

    std::string load_name = std::string("./oldmap/big_ndy_map_mat.bin");
    WriteMat(load_name, big_ndy_map_mat);
}

void DynamicMap::DLoadMat()
{
    FILE *fptl;
    std::string load_center_path = std::string("./oldmap/big_center.txt");
    double *Old_inital;
    int *Old_bei;
    Old_bei = new int [2];
    Old_inital = new double [5];
    //Old_inital = new double [4];
    fptl = fopen(load_center_path.c_str(), "r");
    if(fptl == NULL){
        printf("No Mat Storage, Stop DLoad\n");
        return;
    }
    no_storage_in = false;
    fscanf(fptl, "%lf %lf %lf %lf %lf %d %d\n", Old_inital, Old_inital + 1, Old_inital + 2, Old_inital + 3, Old_inital + 4, Old_bei, Old_bei + 1);
    //fscanf(fptl, "%lf %lf %lf %lf %d %d\n", Old_inital, Old_inital + 1, Old_inital + 2, Old_inital + 3, Old_bei, Old_bei + 1);
    fclose(fptl);
    inital_residual_x = Old_inital[0];
    inital_residual_y = Old_inital[1];
    inital_center_x = Old_inital[2];
    inital_center_y = Old_inital[3];
    reference_height = Old_inital[4];
    Big_x_bei = Old_bei[0];
    Big_y_bei = Old_bei[1];

    std::string load_name = std::string("./oldmap/big_ndy_map_mat.bin");
    big_ndy_map_mat = ReadMat(load_name);
    printf("DLoad old Mat Over! \n");
}


////////////////////////////////////////////// V2 /////////////////////////////////////////////////////////////
void DynamicMap::SetTimes(float dy_times_){
    dy_times = dy_times_;
}

void DynamicMap::PointsInOCV2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr oc_pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::octree::OctreePointCloud<pcl::PointXYZ> current_octree(0.3);
    pcl::PointXYZ pt;
    for(size_t i = 0; i < cloud->size(); i++)
    {
        pt.x = cloud->points[i].x + t_w_l(0);
        pt.y = cloud->points[i].y + t_w_l(1);
        pt.z = cloud->points[i].z + t_w_l(2);
        oc_pts->points.push_back(pt);

        // float dist_to_origin = cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y;
        // if(dist_to_origin>=(roi_range-10)*(roi_range-10))
        //     continue;

        int current_overlap = 0;    //初段检测
        int hole_one = MIN_CLOUDS_IN;
        if(points_in >= MIN_CLOUDS_IN){
            for(auto it = ndy_octree_buffer.begin(); it != ndy_octree_buffer.end(); it++){
            //for(auto it = ndy_octree_buffer.begin() + points_in - MIN_CLOUDS_IN - 1; it != ndy_octree_buffer.end(); it++){
                if((*it).isVoxelOccupiedAtPoint(pt.x, pt.y, pt.z)){
                    current_overlap += hole_one;
                    hole_one--;
                }
            }
        }
        
        int32_t c = map_cols * Big_x_bei / 2 + std::floor((pt.x - inital_center_x - inital_residual_x) / map_resolution);
        int32_t r = map_rows * Big_y_bei / 2 - 1 - std::floor((pt.y - inital_center_y - inital_residual_y) / map_resolution);

        std::lock_guard<std::mutex> lock(mtx_point);
        //CGlobalGridCell *this_current_cell = big_global_ob_map->GetRCLocal(r,c);
        CGlobalGridCellV2 *this_current_cell = big_global_ob_map2->GetRCLocal(r,c);
        this_current_cell->AddObservation(pt.z - reference_height, current_overlap, MIN_CLOUDS_IN);
    }

    current_octree.setInputCloud(oc_pts);
    current_octree.addPointsFromInputCloud();
    ndy_octree_buffer.push_back(current_octree);

    if(points_in >= MIN_CLOUDS_IN)
    {
        //cloud->clear();
        points_in++;
        dy_points->clear();
        ndy_points->clear();
        DynamicVoteOCV2(oc_pts, cloud);
        ndy_octree_buffer.pop_front();
    }else{
        points_in++;
    }

}

void DynamicMap::DynamicVoteOCV2(pcl::PointCloud<pcl::PointXYZ>::Ptr ob_cld, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    int32_t c_offset = map_cols * Big_x_bei / 2 + std::floor((t_w_l(0) - inital_center_x - inital_residual_x) / map_resolution) - map_cols / 2;
    int32_t r_offset = map_rows * Big_y_bei / 2 - 1 - std::floor((t_w_l(1) - inital_center_y - inital_residual_y) / map_resolution) - map_rows / 2;
    ResetIntermediateVariables();

    for(int r=0; r<map_rows; r++){
        for(int c=0; c<map_cols; c++){
            if(r + r_offset<0 || r + r_offset>=map_rows*Big_y_bei || c + c_offset<0 || c + c_offset>=map_cols*Big_x_bei){
                continue;
            }else{
                std::lock_guard<std::mutex> lock(mtx_point);
                int idx = r*map_cols + c;
                CInstantTerrainCell *this_instant_cell = instant_terrain_mapper->instant_terrain_map->cells + idx;
                //CGlobalGridCell *this_cell = big_global_ob_map->GetRCLocal(r + r_offset,c + c_offset);
                CGlobalGridCellV2 *this_cell = big_global_ob_map2->GetRCLocal(r + r_offset,c + c_offset);
                double diff_height = fabs(this_instant_cell->max_height-this_instant_cell->min_height);
                if(this_instant_cell->n > 1 && diff_height < 0.5)
                    this_cell->TerrainObservation();
                // this_cell->OverCheckV2();
                if(check_mode == 1){
                    this_cell->OverCheck();
                }else{
                    this_cell->OverCheckV2();
                }
                //刷新地形数据值
                if(predicted_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT)
                    this_cell->SetTerrainHeight(predicted_terrain_height_mat.at<double>(r,c));

                if(this_cell->ob_min_height == -INVALID_HEIGHT)
                    continue;
                
                if(this_cell->ob_min_height - predicted_terrain_height_mat.at<double>(r,c) < MAX_DYNAMIC){
                    small_ndy_map_mat.at<double>(r,c) = predicted_terrain_height_mat.at<double>(r,c) - DEEP_ABSORT;
                }else{
                    small_ndy_map_mat.at<double>(r,c) = this_cell->ob_min_height - DEEP_ABSORT; 
                }
                //// Mode One ////
                
                this_cell->ReBool();
            }
        }
    }

    bgk_smoother->Run(small_ndy_map_mat, ndy_predict_map_mat, INVALID_HEIGHT, false);
    //VisualizeMat(ndy_predict_map_mat, "predict_term_mat", false);
    VisualizeMatWithInvalidValue(ndy_predict_map_mat, INVALID_HEIGHT, "predict_stable_mat", false);

    pcl::PointXYZ pt;
    for(size_t i=0; i< cloud->points.size(); i++){
        float x = ob_cld->points[i].x;pt.x = cloud->points[i].x;
        float y = ob_cld->points[i].y;pt.y = cloud->points[i].y;
        float z = ob_cld->points[i].z;pt.z = cloud->points[i].z;
        
        int32_t c = map_cols * Big_x_bei / 2 + std::floor((x - inital_center_x - inital_residual_x) / map_resolution) - c_offset;
        int32_t r = map_rows * Big_y_bei / 2 - 1 - std::floor((y - inital_center_y - inital_residual_y) / map_resolution) - r_offset;

        if(r<0 || r>=map_rows || c<0 || c>=map_cols){
            continue;
        }else{
            // if(pt.z - reference_height + r_w_l(2) - predicted_terrain_height_mat.at<double>(r,c) > 3.2){
            //     ndy_points->points.push_back(pt);
            //     continue;
            // }
            if(pt.z > OBSERLUTE_HEIGHT){
                ndy_points->points.push_back(pt);
                continue;
            }

            if(ndy_predict_map_mat.at<double>(r,c) == INVALID_HEIGHT || ndy_predict_map_mat.at<double>(r,c) > z - reference_height){
                dy_points->points.push_back(pt);
            }else{
                ndy_points->points.push_back(pt);
            }
        }
    }   
}

void DynamicMap::PointsInOCV3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr oc_pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::octree::OctreePointCloud<pcl::PointXYZ> current_octree(0.3);
    pcl::PointXYZ pt;
    for(size_t i = 0; i < cloud->size(); i++)
    {
        pt.x = cloud->points[i].x + t_w_l(0);
        pt.y = cloud->points[i].y + t_w_l(1);
        pt.z = cloud->points[i].z + t_w_l(2);
        oc_pts->points.push_back(pt);

        // float dist_to_origin = cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y;
        // if(dist_to_origin>=roi_range*roi_range)
        //     continue;


        int current_overlap = 0;    //初段检测
        int hole_one = MIN_CLOUDS_IN;
        if(points_in >= MIN_CLOUDS_IN){
            for(auto it = ndy_octree_buffer.begin(); it != ndy_octree_buffer.end(); it++){
            //for(auto it = ndy_octree_buffer.begin() + points_in - MIN_CLOUDS_IN - 1; it != ndy_octree_buffer.end(); it++){
                if((*it).isVoxelOccupiedAtPoint(pt.x, pt.y, pt.z)){
                    current_overlap += hole_one;
                    hole_one--;
                }
            }
        }
        
        int32_t c = map_cols * Big_x_bei / 2 + std::floor((pt.x - inital_center_x - inital_residual_x) / map_resolution);
        int32_t r = map_rows * Big_y_bei / 2 - 1 - std::floor((pt.y - inital_center_y - inital_residual_y) / map_resolution);

        std::lock_guard<std::mutex> lock(mtx_point);
        //CGlobalGridCell *this_current_cell = big_global_ob_map->GetRCLocal(r,c);
        CGlobalGridCellV2 *this_current_cell = big_global_ob_map2->GetRCLocal(r,c);
        this_current_cell->AddObservation(pt.z - reference_height, current_overlap, MIN_CLOUDS_IN);
    }

    current_octree.setInputCloud(oc_pts);
    current_octree.addPointsFromInputCloud();
    ndy_octree_buffer.push_back(current_octree);

    if(points_in >= MIN_CLOUDS_IN)
    {
        //cloud->clear();
        points_in++;
        dy_points->clear();
        ndy_points->clear();
        DynamicVoteOCV3(oc_pts, cloud);
        ndy_octree_buffer.pop_front();
    }else{
        points_in++;
    }

    
}

void DynamicMap::DynamicVoteOCV3(pcl::PointCloud<pcl::PointXYZ>::Ptr ob_cld, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    int32_t c_offset = map_cols * Big_x_bei / 2 + std::floor((t_w_l(0) - inital_center_x - inital_residual_x) / map_resolution) - map_cols / 2;
    int32_t r_offset = map_rows * Big_y_bei / 2 - 1 - std::floor((t_w_l(1) - inital_center_y - inital_residual_y) / map_resolution) - map_rows / 2;
    ResetIntermediateVariables();

    for(int r=0; r<map_rows; r++){
        for(int c=0; c<map_cols; c++){
            if(r + r_offset<0 || r + r_offset>=map_rows*Big_y_bei || c + c_offset<0 || c + c_offset>=map_cols*Big_x_bei){
                continue;
            }else{
                std::lock_guard<std::mutex> lock(mtx_point);
                //CGlobalGridCell *this_cell = big_global_ob_map->GetRCLocal(r + r_offset,c + c_offset);
                CGlobalGridCellV2 *this_cell = big_global_ob_map2->GetRCLocal(r + r_offset,c + c_offset);
                // this_cell->OverCheck();
                if(check_mode == 1){
                    this_cell->OverCheck();
                }else{
                    this_cell->OverCheckV2();
                }
                if(this_cell->ob_min_height == -INVALID_HEIGHT)
                    continue;
                
                if(this_cell->ob_min_height - big_terrain_map_mat.at<double>(r + r_offset,c + c_offset) < MAX_DYNAMIC){
                    small_ndy_map_mat.at<double>(r,c) = big_terrain_map_mat.at<double>(r + r_offset,c + c_offset) - DEEP_ABSORT;
                }else{
                    small_ndy_map_mat.at<double>(r,c) = this_cell->ob_min_height - DEEP_ABSORT; 
                }
                //// Mode Two ////                
                this_cell->ReBool();
            }
        }
    }

    bgk_smoother->Run(small_ndy_map_mat, ndy_predict_map_mat, INVALID_HEIGHT, false);
    //VisualizeMat(ndy_predict_map_mat, "predict_term_mat", false);
    VisualizeMatWithInvalidValue(ndy_predict_map_mat, INVALID_HEIGHT, "predict_stable_mat", false);

    pcl::PointXYZ pt;
    for(size_t i=0; i< cloud->points.size(); i++){
        float x = ob_cld->points[i].x;pt.x = cloud->points[i].x;
        float y = ob_cld->points[i].y;pt.y = cloud->points[i].y;
        float z = ob_cld->points[i].z;pt.z = cloud->points[i].z;
        
        int32_t c = map_cols * Big_x_bei / 2 + std::floor((x - inital_center_x - inital_residual_x) / map_resolution) - c_offset;
        int32_t r = map_rows * Big_y_bei / 2 - 1 - std::floor((y - inital_center_y - inital_residual_y) / map_resolution) - r_offset;

        if(r<0 || r>=map_rows || c<0 || c>=map_cols){
            continue;
        }else{
            // if(pt.z - reference_height + r_w_l(2) - big_terrain_map_mat.at<double>(r + r_offset,c + c_offset) > 3.2){
            //     ndy_points->points.push_back(pt);
            //     continue;
            // }
            if(ndy_predict_map_mat.at<double>(r,c) == INVALID_HEIGHT || ndy_predict_map_mat.at<double>(r,c) > z - reference_height){
                dy_points->points.push_back(pt);
            }else{
                ndy_points->points.push_back(pt);
            }
        }
    }   
}

void DynamicMap::InitializeV2()
{
    //big_global_ob_map = new CExpandableMap<CGlobalGridCell>(map_resolution, map_rows * Big_y_bei, map_cols * Big_x_bei);
    big_global_ob_map2 = new CExpandableMap<CGlobalGridCellV2>(map_resolution, map_rows * Big_y_bei, map_cols * Big_x_bei);

    DLoadMatV2();

    big_ndy_map_mat = cv::Mat::zeros(map_rows * Big_y_bei, map_cols * Big_x_bei, CV_64FC1) - INVALID_HEIGHT;
    big_ndy_term_map_mat = cv::Mat::zeros(map_rows * Big_y_bei, map_cols * Big_x_bei, CV_64FC1);
    ndy_predict_map_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    small_ndy_map_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    predicted_terrain_height_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;

    pn_test_model = new PNtestModel();
    dynamic_big_mat = cv::Mat::zeros(map_rows * Big_y_bei, map_cols * Big_x_bei, CV_8UC3);
    dynamic_big_mat = cv::imread("./oldmap/labeled_dynamic_mat.png");
}

void DynamicMap::SetTwlRwlV2(Eigen::Vector3d T_w_l, Eigen::Matrix3d R_w_l)
{
    t_w_l(0) = T_w_l(0);
    t_w_l(1) = T_w_l(1);
    t_w_l(2) = T_w_l(2);
    r_w_l(0) = R_w_l(0);
    r_w_l(1) = R_w_l(1);
    r_w_l(2) = R_w_l(2);
    //current_center_x = t_w_l(0);
    //current_center_y = t_w_l(1);
    current_residual_x = std::floor(t_w_l(0) / map_resolution) * map_resolution - t_w_l(0);
    current_residual_y = std::floor(t_w_l(1) / map_resolution) * map_resolution - t_w_l(1);
    if(points_in == 0 && no_storage_in){
        inital_residual_x = std::floor(t_w_l(0) / map_resolution) * map_resolution - t_w_l(0);
        inital_residual_y = std::floor(t_w_l(1) / map_resolution) * map_resolution - t_w_l(1);
        inital_center_x = t_w_l(0);
        inital_center_y = t_w_l(1);
        //big_global_ob_map->SetCenter(inital_center_x, inital_center_y);
        //big_global_ob_map->SetResidual(inital_residual_x, inital_residual_y);
        big_global_ob_map2->SetCenter(inital_center_x, inital_center_y);
        big_global_ob_map2->SetResidual(inital_residual_x, inital_residual_y);
    }
    OverBoundaryCheckV2();
}

void DynamicMap::OverBoundaryCheckV2()
{
    double dist_x = t_w_l(0) - inital_center_x;
    double dist_y = t_w_l(1) - inital_center_y;
    double x_max_b = map_resolution * map_cols * (Big_x_bei - 1) / 2;
    double y_max_b = map_resolution * map_rows * (Big_y_bei - 1) / 2;
    if(dist_x >= x_max_b){
        ChangeBigMatV2(1);
        inital_center_x += map_resolution * map_cols;
    }else if(dist_x <= -x_max_b){
        ChangeBigMatV2(2);
        inital_center_x -= map_resolution * map_cols;
    }else if(dist_y >= y_max_b){
        ChangeBigMatV2(3);
        inital_center_y += map_resolution * map_rows;
    }else if(dist_y <= -y_max_b){
        ChangeBigMatV2(4);
        inital_center_y -= map_resolution * map_rows;
    }else{
        return;
    }
    LoadMatV2();
}

void DynamicMap::ChangeBigMatV2(int flag)
{
    mtx_point.lock();

    int new_big_x = Big_x_bei, new_big_y = Big_y_bei;
    int c_offset = 0;
    int r_offset = 0;

    switch (flag)
    {
    case 1:
    case 2:
        new_big_x = Big_x_bei + 2;
        c_offset = flag - 1;
        c_offset = c_offset * 2 * map_cols;
        break;
    case 3:
    case 4:
        new_big_y = Big_y_bei + 2;
        r_offset = 4 - flag;
        r_offset = r_offset * 2 * map_rows;
        break;
    default:
        break;
    }
    //cv::Mat new_big_ndy_map_mat = cv::Mat::zeros(map_rows * new_big_y, map_cols * new_big_x, CV_64FC1) - INVALID_HEIGHT;
    //CExpandableMap<CGlobalGridCell> *new_big_global_ob_map = new CExpandableMap<CGlobalGridCell>(map_resolution, map_rows * new_big_y, map_cols * new_big_x);
    CExpandableMap<CGlobalGridCellV2> *new_big_global_ob_map2 = new CExpandableMap<CGlobalGridCellV2>(map_resolution, map_rows * new_big_y, map_cols * new_big_x);
    big_ndy_map_mat = cv::Mat::zeros(map_rows * new_big_y, map_cols * new_big_x, CV_64FC1) - INVALID_HEIGHT;
    big_ndy_term_map_mat = cv::Mat::zeros(map_rows * new_big_y, map_cols * new_big_x, CV_64FC1);


    for(int r = 0; r < map_rows * Big_y_bei; r++){
        for(int c = 0; c < map_cols * Big_x_bei; c++){
            CGlobalGridCellV2 *this_new_cell = new_big_global_ob_map2->GetRCLocal(r + r_offset,c + c_offset);
            CGlobalGridCellV2 *this_old_cell = big_global_ob_map2->GetRCLocal(r,c);
            // CGlobalGridCell *this_new_cell = new_big_global_ob_map->GetRCLocal(r + r_offset,c + c_offset);
            // CGlobalGridCell *this_old_cell = big_global_ob_map->GetRCLocal(r,c);
            this_new_cell->ob_min_height = this_old_cell->ob_min_height;
            this_new_cell->n = this_old_cell->n;
            this_new_cell->terrain_height = this_old_cell->terrain_height;
        }
    }
    // big_global_ob_map = new CExpandableMap<CGlobalGridCell>(map_resolution, map_rows * new_big_y, map_cols * new_big_x);
    // big_global_ob_map = new_big_global_ob_map;
    big_global_ob_map2 = new CExpandableMap<CGlobalGridCellV2>(map_resolution, map_rows * new_big_y, map_cols * new_big_x);
    big_global_ob_map2 = new_big_global_ob_map2;
    //big_ndy_map_mat = new_big_ndy_map_mat.clone();
    Big_x_bei = new_big_x;
    Big_y_bei = new_big_y;
    mtx_point.unlock();
}

void DynamicMap::LoadMatV2()
{
    FILE *fptl;
    std::string load_center_path = std::string("./oldmap/big_center.txt");
    fptl = fopen(load_center_path.c_str(), "w");
    fprintf(fptl, "%lf %lf %f %f %f %d %d\n", inital_residual_x, inital_residual_y, inital_center_x, inital_center_y, reference_height, Big_x_bei, Big_y_bei);
    fclose(fptl);
    for(int r = 0; r < map_rows * Big_y_bei; r++){
        for(int c = 0; c < map_cols * Big_x_bei; c++){
            std::lock_guard<std::mutex> lock(mtx_point);
            //CGlobalGridCell *this_cell = big_global_ob_map->GetRCLocal(r,c);
            CGlobalGridCellV2 *this_cell = big_global_ob_map2->GetRCLocal(r,c);
            big_ndy_map_mat.at<double>(r,c) = this_cell->ob_min_height;
            big_ndy_term_map_mat.at<double>(r,c) = this_cell->n;
        }
    }
    std::string load_name = std::string("./oldmap/big_ndy_map_mat.bin");
    WriteMat(load_name, big_ndy_map_mat);

    load_name = std::string("./oldmap/big_ndy_term_map_mat.bin");
    WriteMat(load_name, big_ndy_term_map_mat);


    // int32_t c_offset = map_cols * Big_x_bei / 2 + std::floor((t_w_l(0) - inital_center_x - inital_residual_x) / map_resolution) - map_cols / 2;
    // int32_t r_offset = map_rows * Big_y_bei / 2 - 1 - std::floor((t_w_l(1) - inital_center_y - inital_residual_y) / map_resolution) - map_rows / 2;

    //build dynamic map
    // pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_big_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile("/home/wswork/文档/label_all_dy_cloud.pcd", *dynamic_big_cloud);
    // for(size_t i = 0; i < dynamic_big_cloud->points.size(); i++){
    //     float x = dynamic_big_cloud->points[i].x;
    //     float y = dynamic_big_cloud->points[i].y;

    //     int32_t c_this = map_cols * Big_x_bei / 2 + std::floor((x - inital_center_x - inital_residual_x) / map_resolution);
    //     int32_t r_this = map_rows * Big_y_bei / 2 - 1 - std::floor((y - inital_center_y - inital_residual_y) / map_resolution);

    //     dynamic_big_mat.at<cv::Vec3b>(r_this, c_this) = cv::Vec3b(0, 0, 255);
    // }

    // cv::imwrite("./oldmap/labeled_dynamic_mat.png", dynamic_big_mat);
}

void DynamicMap::LoadBigTerrainMat()
{
    big_terrain_map_mat = cv::Mat::zeros(map_rows * Big_y_bei, map_cols * Big_x_bei, CV_64FC1) + INVALID_HEIGHT;
    for(int r = 0; r < map_rows * Big_y_bei; r++){
        for(int c = 0; c < map_cols * Big_x_bei; c++){
            std::lock_guard<std::mutex> lock(mtx_point);
            //CGlobalGridCell *this_cell = big_global_ob_map->GetRCLocal(r,c);
            CGlobalGridCellV2 *this_cell = big_global_ob_map2->GetRCLocal(r,c);
            big_terrain_map_mat.at<double>(r,c) = this_cell->terrain_height;
        }
    }
    std::string load_name = std::string("./oldmap/big_terrain_map_mat.bin");
    WriteMat(load_name, big_terrain_map_mat);
}

void DynamicMap::DLoadBigTerrainMat()
{
    big_terrain_map_mat = cv::Mat::zeros(map_rows * Big_y_bei, map_cols * Big_x_bei, CV_64FC1) + INVALID_HEIGHT;
    std::string load_name = std::string("./oldmap/big_terrain_map_mat.bin");
    big_terrain_map_mat = ReadMat(load_name);
}

void DynamicMap::DLoadMatV2()
{
    FILE *fptl;
    std::string load_center_path = std::string("./oldmap/big_center.txt");
    double *Old_inital;
    int *Old_bei;
    Old_bei = new int [2];
    Old_inital = new double [5];
    //Old_inital = new double [4];
    fptl = fopen(load_center_path.c_str(), "r");
    if(fptl == NULL){
        printf("No Mat Storage, Stop DLoad\n");
        return;
    }
    no_storage_in = false;
    fscanf(fptl, "%lf %lf %lf %lf %lf %d %d\n", Old_inital, Old_inital + 1, Old_inital + 2, Old_inital + 3, Old_inital + 4, Old_bei, Old_bei + 1);
    //fscanf(fptl, "%lf %lf %lf %lf %d %d\n", Old_inital, Old_inital + 1, Old_inital + 2, Old_inital + 3, Old_bei, Old_bei + 1);
    fclose(fptl);
    inital_residual_x = Old_inital[0];
    inital_residual_y = Old_inital[1];
    inital_center_x = Old_inital[2];
    inital_center_y = Old_inital[3];
    reference_height = Old_inital[4];
    Big_x_bei = Old_bei[0];
    Big_y_bei = Old_bei[1];

    //big_global_ob_map = new CExpandableMap<CGlobalGridCell>(map_resolution, map_rows * Big_y_bei, map_cols * Big_x_bei);
    big_global_ob_map2 = new CExpandableMap<CGlobalGridCellV2>(map_resolution, map_rows * Big_y_bei, map_cols * Big_x_bei);

    std::string load_name = std::string("./oldmap/big_ndy_map_mat.bin");
    big_ndy_map_mat = ReadMat(load_name);
    load_name = std::string("./oldmap/big_ndy_term_map_mat.bin");
    big_ndy_term_map_mat = ReadMat(load_name);

    for(int r = 0; r < map_rows * Big_y_bei; r++){
        for(int c = 0; c < map_cols * Big_x_bei; c++){
            //CGlobalGridCell *this_cell = big_global_ob_map->GetRCLocal(r,c);
            CGlobalGridCellV2 *this_cell = big_global_ob_map2->GetRCLocal(r,c);
            this_cell->ob_min_height = big_ndy_map_mat.at<double>(r,c);
            this_cell->n = big_ndy_term_map_mat.at<double>(r,c);
        }
    }
    //big_global_ob_map->SetCenter(inital_center_x, inital_center_y);
    //big_global_ob_map->SetResidual(inital_residual_x, inital_residual_y);
    big_global_ob_map2->SetCenter(inital_center_x, inital_center_y);
    big_global_ob_map2->SetResidual(inital_residual_x, inital_residual_y);
    printf("DLoad old Mat Over! \n");
}


void DynamicMap::VisualizeDynamicMap(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, bool b_pause)
{
    if (!GetVisualizationEnabled()) {
        return;
    }
    if(vis==NULL)
        vis = new pcl::visualization::PCLVisualizer ("VisualizeDynamicMap");

    vis->removeAllPointClouds();
    //vis->removeAllShapes();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB pt;

    for(size_t i=0; i<point_cloud->size(); i++) {
        pt.x = point_cloud->points[i].x;
        pt.y = point_cloud->points[i].y;
        pt.z = point_cloud->points[i].z;
        pt.r = 0;
        pt.g = 200;
        pt.b = 80;
        colored_cloud->push_back(pt);
    }

    for(size_t i=0; i<ndy_points->size(); i++) {
        pt.x = ndy_points->points[i].x;
        pt.y = ndy_points->points[i].y;
        pt.z = ndy_points->points[i].z;
        pt.r = 0;
        pt.g = 200;
        pt.b = 80;
        colored_cloud->push_back(pt);
    }

    for(size_t i=0; i<dy_points->size(); i++) {
        pt.x = dy_points->points[i].x;
        pt.y = dy_points->points[i].y;
        pt.z = dy_points->points[i].z;
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        colored_cloud->push_back(pt);
    }
    
    vis->addPointCloud (colored_cloud, "cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    if(b_pause)
        vis->spin();
    else
        vis->spinOnce(10);
}

void DynamicMap::Output_splot()
{
    FILE *fptl;
    std::string load_center_path = std::string("./oldmap/splot_data.txt");
    fptl = fopen(load_center_path.c_str(), "a");
    fprintf(fptl, "%lf %lf %lf %lf\n", splot[0] + splot[1], (splot[3] - splot[2])*1000, splot[4], splot[5]);
    fclose(fptl);
}

void DynamicMap::TestModel_All(cv::Mat &test_model, int offset_r, int offset_c)
{
    pn_test_model->ReSetThis();
    for(int r = 0; r < map_rows; r++){
        for(int c = 0; c < map_cols; c++){
            if(dynamic_big_mat.at<cv::Vec3b>(r + offset_r, c + offset_c) == cv::Vec3b(0, 0, 0)){
                continue;
            }
            if(dynamic_big_mat.at<cv::Vec3b>(r + offset_r, c + offset_c) == cv::Vec3b(0, 0, 255)){
                if(test_model.at<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 255)){
                    pn_test_model->AddObservation(0);
                }else if(test_model.at<cv::Vec3b>(r, c) == cv::Vec3b(0, 255, 0)){
                    pn_test_model->AddObservation(3);
                }
            }else if(dynamic_big_mat.at<cv::Vec3b>(r + offset_r, c + offset_c) == cv::Vec3b(0, 255, 0)){
                if(test_model.at<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 255)){
                    pn_test_model->AddObservation(2);
                }else if(test_model.at<cv::Vec3b>(r, c) == cv::Vec3b(0, 255, 0)){
                    pn_test_model->AddObservation(1);
                }
            }
        }
    }

    printf("Precision_this: %f -- Recall_this: %f \n", pn_test_model->ReturnPrecision_this(), pn_test_model->ReturnRecall_this());
    if(pn_test_model->Fn_this + pn_test_model->Fp_this + pn_test_model->Tp_this + pn_test_model->Tn_this > 0)
        printf("ACC_this: %f -- Fpositive_this: %f \n", pn_test_model->ReturnACC_this(), pn_test_model->ReturnFalsePositiveRate_this());

    printf("Precision_all: %f -- Recall_all: %f \n", pn_test_model->ReturnPrecision_total(), pn_test_model->ReturnRecall_total());
    if(pn_test_model->Fn_total + pn_test_model->Fp_total + pn_test_model->Tp_total + pn_test_model->Tn_total > 0)
        printf("ACC_all: %f -- Fpositive_all: %f \n", pn_test_model->ReturnACC_total(), pn_test_model->ReturnFalsePositiveRate_total());
}

void DynamicMap::OutPutMap(){
    char load_name[100];
    sprintf(load_name, "./OutputMap/dy_ours.pcd");
    pcl::io::savePCDFileBinary(std::string(load_name), *dy_points_tmp);
    sprintf(load_name, "./OutputMap/ndy_ours.pcd");
    pcl::io::savePCDFileBinary(std::string(load_name), *ndy_points_tmp);
    printf("Save Map Done!\n");
}

void DynamicMap::OutPutMap(std::string output_path){
    char load_name[100];
    const std::filesystem::path output_root = ResolveOutputRoot(output_path);
    // sprintf(load_name, "%s/OutputMap/dy_ours.pcd", output_path.c_str());
    // pcl::io::savePCDFileBinary(std::string(load_name), *dy_points_tmp);
    sprintf(load_name, "%s/pgp_output.pcd", output_root.string().c_str());
    pcl::io::savePCDFileBinary(std::string(load_name), *ndy_points_tmp);
    printf("Save Map Done!\n");
}

void DynamicMap::VisualizeDynamicMatkitti(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_terrain, std::string output_path)
{
    const std::filesystem::path output_root = ResolveOutputRoot(output_path);
    nums_vis++;
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;

    cv::Mat mat = cv::Mat::zeros(map_rows, map_cols, CV_8UC3);
    cv::Mat dis_mat = cv::Mat::zeros(map_rows, map_cols, CV_8UC3);
    int32_t c_offset = map_cols * Big_x_bei / 2 + std::floor((t_w_l(0) - inital_center_x - inital_residual_x) / map_resolution) - map_cols / 2;
    int32_t r_offset = map_rows * Big_y_bei / 2 - 1 - std::floor((t_w_l(1) - inital_center_y - inital_residual_y) / map_resolution) - map_rows / 2;

    pcl::PointXYZRGB pt;

    for(size_t i=0; i<dy_points->size(); i++) {
        float x = dy_points->points[i].x + t_w_l(0);pt.x = x - offset_x;
        float y = dy_points->points[i].y + t_w_l(1);pt.y = y - offset_y;

        pt.z = dy_points->points[i].z + t_w_l(2) - offset_z;
        pt.r = 0;
        pt.g = 0;
        pt.b = 255;

        dy_points_tmp->push_back(pt);
        
        int32_t c_this = map_cols * Big_x_bei / 2 + std::floor((x - inital_center_x - inital_residual_x) / map_resolution) - c_offset;
        int32_t r_this = map_rows * Big_y_bei / 2 - 1 - std::floor((y - inital_center_y - inital_residual_y) / map_resolution) - r_offset;

        if(r_this<0 || r_this>=map_rows || c_this<0 || c_this>=map_cols){
            continue;
        }else{
            mat.at<cv::Vec3b>(r_this, c_this) = cv::Vec3b(0, 0, 255);
            dis_mat.at<cv::Vec3b>(r_this, c_this) = cv::Vec3b(0, 0, 255);
        }
    }
    
    char load_name[400];
    sprintf(load_name, "%s/result_pgp/%.4d.png", output_root.string().c_str(), start_frame + MIN_CLOUDS_IN);
    start_frame++;

    cv::imwrite(std::string(load_name), mat);

    // 非动态点
    for(size_t i=0; i<ndy_points->size(); i++) {
        float x = ndy_points->points[i].x + t_w_l(0);pt.x = x - offset_x;
        float y = ndy_points->points[i].y + t_w_l(1);pt.y = y - offset_y;

        pt.z = ndy_points->points[i].z + t_w_l(2) - offset_z;

        pt.r = 255;
        pt.g = 0;
        pt.b = 0;

        ndy_points_tmp->push_back(pt);
        
        int32_t c_this = map_cols * Big_x_bei / 2 + std::floor((x - inital_center_x - inital_residual_x) / map_resolution) - c_offset;
        int32_t r_this = map_rows * Big_y_bei / 2 - 1 - std::floor((y - inital_center_y - inital_residual_y) / map_resolution) - r_offset;

        if(r_this<0 || r_this>=map_rows || c_this<0 || c_this>=map_cols){
            continue;
        }else{
            //mat.at<cv::Vec3b>(r_this, c_this) = cv::Vec3b(0, 0, 20);
            dis_mat.at<cv::Vec3b>(r_this, c_this) = cv::Vec3b(0, 255, 0);
        }
    }

    for(size_t i=0; i<point_cloud_terrain->size(); i++) {
        float x = point_cloud_terrain->points[i].x + t_w_l(0);pt.x = x - offset_x;
        float y = point_cloud_terrain->points[i].y + t_w_l(1);pt.y = y - offset_y;

        pt.z = point_cloud_terrain->points[i].z + t_w_l(2) - offset_z;

        pt.r = 255;
        pt.g = 0;
        pt.b = 0;

        ndy_points_tmp->push_back(pt);
    }

    // 显示标记后的图像
    if (GetVisualizationEnabled()) {
        cv::imshow("Tracked Dynamic Targets", dis_mat);
        // if(nums_vis < 500){
            cv::waitKey(1); // 等待1ms并刷新图像窗口
        // }else{
        //     cv::waitKey(-1); // 等待1ms并刷新图像窗口
        // }
    }
}

void DynamicMap::VisualizeDynamicMat(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_terrain)
{
    // double offset_x = -0.031042;
    // double offset_y = -0.182821;
    // double offset_z = -0.201462;

    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;

    // double offset_x = -12332.492695;
    // double offset_y = -12881.060450;
    // double offset_z = 71.111108;

    cv::Mat mat = cv::Mat::zeros(map_rows, map_cols, CV_8UC3);
    cv::Mat dis_mat = cv::Mat::zeros(map_rows, map_cols, CV_8UC3);
    int32_t c_offset = map_cols * Big_x_bei / 2 + std::floor((t_w_l(0) - inital_center_x - inital_residual_x) / map_resolution) - map_cols / 2;
    int32_t r_offset = map_rows * Big_y_bei / 2 - 1 - std::floor((t_w_l(1) - inital_center_y - inital_residual_y) / map_resolution) - map_rows / 2;

    pcl::PointXYZRGB pt;

    for(size_t i=0; i<dy_points->size(); i++) {
        float x = dy_points->points[i].x + t_w_l(0);pt.x = x - offset_x;
        float y = dy_points->points[i].y + t_w_l(1);pt.y = y - offset_y;

        pt.z = dy_points->points[i].z + t_w_l(2) - offset_z;
        pt.r = 0;
        pt.g = 0;
        pt.b = 255;

        dy_points_tmp->push_back(pt);
        
        int32_t c_this = map_cols * Big_x_bei / 2 + std::floor((x - inital_center_x - inital_residual_x) / map_resolution) - c_offset;
        int32_t r_this = map_rows * Big_y_bei / 2 - 1 - std::floor((y - inital_center_y - inital_residual_y) / map_resolution) - r_offset;

        if(r_this<0 || r_this>=map_rows || c_this<0 || c_this>=map_cols){
            continue;
        }else{
            mat.at<cv::Vec3b>(r_this, c_this) = cv::Vec3b(0, 0, 255);
            dis_mat.at<cv::Vec3b>(r_this, c_this) = cv::Vec3b(0, 0, 255);
        }
    }
    
    char load_name[100];
    sprintf(load_name, "./Mat/%ld.png", timestamp_this);

    // 非动态点
    for(size_t i=0; i<ndy_points->size(); i++) {
        float x = ndy_points->points[i].x + t_w_l(0);pt.x = x - offset_x;
        float y = ndy_points->points[i].y + t_w_l(1);pt.y = y - offset_y;

        pt.z = ndy_points->points[i].z + t_w_l(2) - offset_z;

        pt.r = 255;
        pt.g = 0;
        pt.b = 0;

        ndy_points_tmp->push_back(pt);
        
        int32_t c_this = map_cols * Big_x_bei / 2 + std::floor((x - inital_center_x - inital_residual_x) / map_resolution) - c_offset;
        int32_t r_this = map_rows * Big_y_bei / 2 - 1 - std::floor((y - inital_center_y - inital_residual_y) / map_resolution) - r_offset;

        if(r_this<0 || r_this>=map_rows || c_this<0 || c_this>=map_cols){
            continue;
        }else{
            //mat.at<cv::Vec3b>(r_this, c_this) = cv::Vec3b(0, 0, 20);
            dis_mat.at<cv::Vec3b>(r_this, c_this) = cv::Vec3b(0, 255, 0);
        }
    }

    for(size_t i=0; i<point_cloud_terrain->size(); i++) {
        float x = point_cloud_terrain->points[i].x + t_w_l(0);pt.x = x - offset_x;
        float y = point_cloud_terrain->points[i].y + t_w_l(1);pt.y = y - offset_y;

        pt.z = point_cloud_terrain->points[i].z + t_w_l(2) - offset_z;

        pt.r = 255;
        pt.g = 0;
        pt.b = 0;

        ndy_points_tmp->push_back(pt);
    }

    // cv::Mat slope_roi_dilated = cv::Mat::zeros(map_rows, map_cols, CV_8UC3);

    // // 创建一个方块形状的结构元素，尺寸为3x3
    // cv::Mat erode3 = cv::Mat::zeros(map_rows, map_cols, CV_8UC3);
    // cv::Mat element_dil = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(4, 4));
    // cv::Mat element_slope = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    // // 对检测到的陡坡区域进行膨胀操作
    // cv::erode(mat, erode3, element_slope, cv::Point(-1, -1), 1);
    // cv::dilate(erode3, slope_roi_dilated, element_slope, cv::Point(-1, -1), 3);

    // //cv::Mat filter_mat =  preprocessImage(mat);
    // // 初始化
    // std::vector<cv::Rect> target_boxes;
    // std::vector<cv::Point2f> target_centers;
    // std::vector<cv::Point2f> previous_centers;
    // std::vector<cv::Point2f> velocities;
    // std::vector<float> directions;

    

    // // 处理当前帧
    // processFrame(slope_roi_dilated, target_boxes, target_centers);

    // // 计算目标速度和方向
    // for (size_t i = 0; i < target_centers.size(); i++) {
    //     if (i < previous_centers.size()) {
    //         // 计算速度和方向
    //         cv::Point2f velocity = calculateVelocity(previous_centers[i], target_centers[i], 1.0f); // 假设时间间隔为1
    //         velocities.push_back(velocity);
    //         directions.push_back(calculateDirection(velocity));
    //     }

    //     // 更新前一帧的目标位置
    //     previous_centers.push_back(target_centers[i]);
    // }

    // // 可视化结果
    // //visualizeResult(mat, target_boxes, velocities, directions);

    // // 确保目标框有效
    // if (target_boxes.empty())
    // {
    //     std::cerr << "Error: No target boxes detected!" << std::endl;
    // }else{
    //     for (size_t i = 0; i < target_boxes.size(); i++)
    //     {
    //         // 创建一个副本来修改
    //         cv::Rect box = target_boxes[i];

    //         // 确保矩形的宽度和高度是正的
    //         if (box.width <= 0 || box.height <= 0)
    //         {
    //             std::cerr << "Error: Invalid rectangle dimensions at index " << i << std::endl;
    //             continue; // 跳过无效的矩形
    //         }

    //         // 确保矩形坐标在图像范围内
    //         if (box.x < 0)
    //             box.x = 0;
    //         if (box.y < 0)
    //             box.y = 0;
    //         if (box.x + box.width > mat.cols)
    //             box.width = mat.cols - box.x;
    //         if (box.y + box.height > mat.rows)
    //             box.height = mat.rows - box.y;

    //         // 绘制矩形（红色，线宽2）
    //         cv::rectangle(dis_mat, box, cv::Scalar(0, 0, 255), 2);

    //         printf("Target %d: (%d, %d, %d, %d)\n", i, box.x, box.y, box.width, box.height);
    //         printf("velocites size : %d\n", velocities.size());

    //         // 转换为实际速度单位，并显示速度和方向
    //         float speed = 0.0;
    //         if (velocities.size() > 0)
    //         {
    //             speed = cv::norm(velocities[i]); // 像素单位
    //         }
    //         else
    //         {
    //             speed = 0.0;
    //         }

    //         float actual_speed = speed * 0.2f; // 转换为米/秒

    //         printf("speed : %f\n");

    //         cv::putText(dis_mat, "Target " + std::to_string(i), target_boxes[i].tl(),
    //                     cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
    //         if (directions.size() > 0)
    //         {
    //             cv::putText(dis_mat, "Direction: " + std::to_string(directions[i]) + " degrees", target_boxes[i].br(),
    //                         cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
    //         }
    //     }
    // }

    

    // 显示标记后的图像
    if (GetVisualizationEnabled()) {
        cv::imshow("Tracked Dynamic Targets", dis_mat);
        cv::waitKey(1); // 等待1ms并刷新图像窗口
    }


    // cv::imwrite(std::string(load_name), mat);

    

    // for(int r = 0; r < map_rows; r++){
    //     for(int c = 0; c < map_cols; c++){
    //         if(ndy_predict_map_mat.at<double>(r,c) != INVALID_HEIGHT){
    //             mat.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 255, 0);
    //         }
    //     }
    // }


    //build dynamic map
    // for(int r = 0; r < map_rows; r++){
    //     for(int c = 0; c < map_cols; c++){
    //         if(mat.at<cv::Vec3b>(r, c) == cv::Vec3b(0, 0, 0))
    //             continue;
    //         if(dynamic_big_mat.at<cv::Vec3b>(r + r_offset, c + c_offset) == cv::Vec3b(0, 255, 0))
    //             continue;

    //         dynamic_big_mat.at<cv::Vec3b>(r + r_offset, c + c_offset) = mat.at<cv::Vec3b>(r, c);
    //         //dynamic_big_mat.at<cv::Vec3b>(r + r_offset, c + c_offset) = cv::Vec3b(0, 255, 0);
    //     }
    // }

    //TestModel_All(mat, r_offset, c_offset);

    // cv::imshow("DynamicMat", mat);
    // cv::waitKey(2);
}
