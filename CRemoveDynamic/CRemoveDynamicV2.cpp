#include "CRemoveDynamicV2.h"

RemoveDynamicV2::RemoveDynamicV2()
{
    vis = NULL;
    b_enable_visualization = false;
}

RemoveDynamicV2::~RemoveDynamicV2()
{

}

void RemoveDynamicV2::SetLiDARDataPath(std::string LiDARDataPath_)
{
    LiDARDataPath = LiDARDataPath_;
}

void RemoveDynamicV2::SetParamFile(std::string param_filename_)
{
    param_filename = param_filename_;
}

void RemoveDynamicV2::ReadPara()
{
    std::ifstream fin(param_filename.c_str());
    //fin.open(param_filename.c_str());
    if (fin.is_open() != 1) {
        std::cout << "Fail to open params file: " << param_filename << std::endl;
        abort();
    }
    std::string t_s;
    while (fin >> t_s) {
        if (t_s[0] == '#' || t_s[0] == '/')
            std::getline(fin, t_s);
        else if (t_s == "roi_range")
            fin >> roi_range;
        else if (t_s == "map_resolution")
            fin >> map_resolution;
        else if (t_s == "map_cols")
            fin >> map_cols;
        else if (t_s == "k_prediction_kernel_radius")
            fin >> k_prediction_kernel_radius;
        else if (t_s == "k_prediction_kernel_radius_s")
            fin >> k_prediction_kernel_radius_s;
        else if (t_s == "k_valid_points_num_thresh")
            fin >> k_valid_points_num_thresh;
        else if (t_s == "k_valid_points_num_thresh_s")
            fin >> k_valid_points_num_thresh_s;
        else if (t_s == "k_bf_height_std")
            fin >> k_bf_height_std;
        else if (t_s == "RONI_min_x")
            fin >> RONI_min_x;
        else if (t_s == "RONI_max_x")
            fin >> RONI_max_x;
        else if (t_s == "RONI_min_y")
            fin >> RONI_min_y;
        else if (t_s == "RONI_max_y")
            fin >> RONI_max_y;
        else if (t_s == "RONI_min_z")
            fin >> RONI_min_z;
        else if (t_s == "RONI_max_z")
            fin >> RONI_max_z; 
        else if (t_s == "Mode")
            fin >> Mode_in; 
        else if (t_s == "chech_mode")
            fin >> chech_mode; 
        else if (t_s == "b_enable_visualization")
            fin >> b_enable_visualization;
    }
    fin.close();
    map_rows = map_cols;
}

void RemoveDynamicV2::Initialize()
{
    ReadPara();
    SetVisualizationEnabled(b_enable_visualization);
    num_cloudin = 0;

    //ikdtree_dy_map.set_downsample_param(0.01);
    //ikdtree_ndy_map.set_downsample_param(0.01);

    //vis = new pcl::visualization::PCLVisualizer ("PointCloud");
    output_dy_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
    output_nody_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    output_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    output_cloud_dy.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    bgk_smoother = new BGK_Smoother;
    bgk_smoother->SetMapResolution(map_resolution);
    bgk_smoother->SetMapSize(map_rows, map_cols);
    bgk_smoother->SetPredictionKernelRadius(k_prediction_kernel_radius);
    bgk_smoother->SetValidPointsNumThresh(k_valid_points_num_thresh);
    bgk_smoother->SetBilateralFilterStdValue(k_bf_height_std);
    bgk_smoother->Initialize();

    high_res_bgk_smoother = new BGK_Smoother;
    high_res_bgk_smoother->SetMapResolution(map_resolution);
    high_res_bgk_smoother->SetMapSize(map_rows, map_cols);
    high_res_bgk_smoother->SetPredictionKernelRadius(k_prediction_kernel_radius_s);
    high_res_bgk_smoother->SetValidPointsNumThresh(k_valid_points_num_thresh_s);
    high_res_bgk_smoother->SetBilateralFilterStdValue(k_bf_height_std);
    high_res_bgk_smoother->Initialize();

    // range_image_stack = new CRangeImageStack;
    // range_image_stack->SetLiDARDataPath(LiDARDataPath);
    // range_image_stack->SetParamFile(param_filename);
    // range_image_stack->Initialize();

    /////////////////////////////////////////////////////////////////////////

    instant_terrain_mapper = new CInstantTerrainMapper();
    accumed_terrain_mapper = new CAccumedTerrainMapper();

    instant_terrain_mapper->SetLiDARDataPath(LiDARDataPath);
    instant_terrain_mapper->SetParamFile(param_filename);
    instant_terrain_mapper->SetBGKSmoother(bgk_smoother);
    instant_terrain_mapper->SetAccumedTerrainMapper(accumed_terrain_mapper);
    instant_terrain_mapper->Initialize();

    accumed_terrain_mapper->SetLiDARDataPath(LiDARDataPath);
    accumed_terrain_mapper->SetParamFile(param_filename);
    accumed_terrain_mapper->SetBGKSmoother(bgk_smoother);
    accumed_terrain_mapper->SetInstantTerrainMapper(instant_terrain_mapper);
    accumed_terrain_mapper->Initialize();

    //////////////////////////////////////////////////////////////////////////

    dynamicmap_rm = new DynamicMap();

    dynamicmap_rm->SetMapResolution(map_resolution);
    dynamicmap_rm->SetMapSize(map_rows, map_cols);
    dynamicmap_rm->SetROIRange(roi_range);
    dynamicmap_rm->SetCheckmode(chech_mode);
    dynamicmap_rm->SetBGKSmoother(high_res_bgk_smoother);
    dynamicmap_rm->SetAccumedTerrainMapper(accumed_terrain_mapper);
    dynamicmap_rm->SetInstantTerrainMapper(instant_terrain_mapper);
    if(Mode_in == 0){
        dynamicmap_rm->Initialize();
    }else if(Mode_in == 1){
        dynamicmap_rm->InitializeV2();
    }else if(Mode_in == 2){
        dynamicmap_rm->InitializeV2();
        dynamicmap_rm->DLoadBigTerrainMat();
    }
    
}

void RemoveDynamicV2::PreprocessPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud)
{
    current_point_cloud->clear();

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for(size_t i=0; i<point_cloud->size(); i++) {
        if(!isnan(point_cloud->points[i].x)) {

            Eigen::Vector3d local_point, global_point;

            // local_point(0) = point_cloud->points[i].x;
            // local_point(1) = point_cloud->points[i].y;
            // local_point(2) = point_cloud->points[i].z;

            global_point(0) = point_cloud->points[i].x;
            global_point(1) = point_cloud->points[i].y;
            global_point(2) = point_cloud->points[i].z;

            // local_point = R_w_l.transpose() * (global_point) - T_w_l;
            local_point = R_w_l.transpose() * (global_point - T_w_l);

            // if(local_point(0)>RONI_min_x && local_point(0)<RONI_max_x && local_point(1)>RONI_min_y &&
            //         local_point(1)<RONI_max_y && local_point(2)>RONI_min_z && local_point(2)<RONI_max_z)  // remove those lidar points that hit on the vehicle itself!
            //     continue;

            float dist_to_origin = local_point(0)*local_point(0) + local_point(1)*local_point(1);
            if(dist_to_origin>1 && dist_to_origin<roi_range*roi_range) {

//                if(local_point(1)<0)        // only process the front part
//                    continue;

                pcl::PointXYZI pt;
                pt.x = local_point(0);
                pt.y = local_point(1);
                pt.z = local_point(2);

                tmp_point_cloud->points.push_back(pt);
            }
        }
    }
    static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(tmp_point_cloud);
    //voxel_filter.setLeafSize(0.3, 0.3, 0.3);
    voxel_filter.setLeafSize(0.2, 0.20, 0.20);
    //voxel_filter.setLeafSize(0.1, 0.1, 0.1);
    voxel_filter.filter(*current_point_cloud);

    // current_point_cloud = point_cloud;
}

void RemoveDynamicV2::Run(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Vector3d t_w_l, Eigen::Matrix3d r_w_l, INT64 timestamp)
{
    num_cloudin++;
    double t = tic();

    //test//
    // dynamicmap_rm->splot[2] = tic();
    R_w_l = r_w_l;
    T_w_l = t_w_l;

    output_dy_pts->clear();
    output_nody_pts->clear();

    static bool b_first_run = true;
    //static float reference_height;
    if(b_first_run) {
        dynamicmap_rm->SetStartFrame(start_frame);
        if(dynamicmap_rm->no_storage_in){
            reference_height = t_w_l(2);
            dynamicmap_rm->SetReferenceHeight(reference_height);
        }else{
            reference_height = dynamicmap_rm->reference_height;
        }
        accumed_terrain_mapper->SetReferenceHeight(reference_height);
        b_first_run = false;
    }
    PreprocessPointCloud(point_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_build(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nterrain_build(new pcl::PointCloud<pcl::PointXYZ>);

    //std::thread td_1(&DynamicMap::SetTwlRwl, dynamicmap_rm, std::ref(t_w_l), std::ref(r_w_l));
    //td_1.join();
    dynamicmap_rm->timestamp_this = timestamp;

    if(Mode_in == 0){
        dynamicmap_rm->SetTwlRwl(t_w_l, r_w_l);
        accumed_terrain_mapper->PreProcess(t_w_l, r_w_l);
        instant_terrain_mapper->Run(current_point_cloud, t_w_l, r_w_l, 0);
        accumed_terrain_mapper->Run(current_point_cloud, 0);

        toc(t);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_build(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_in_build = accumed_terrain_mapper->rebuild_point_cloud_n;
    
        dynamicmap_rm->predicted_terrain_height_mat = accumed_terrain_mapper->accumed_predicted_terrain_height_mat.clone();

        dynamicmap_rm->Addpoint2clouds(cloud_in_build, output_nody_pts);
        //printf("size of rebuild_point_cloud : %ld\n", accumed_terrain_mapper->rebuild_point_cloud->size());
        dynamicmap_rm->PointsInOC(accumed_terrain_mapper->rebuild_point_cloud);
        dynamicmap_rm->Addpoint2clouds(dynamicmap_rm->ndy_points, output_nody_pts);
    }else if(Mode_in == 1){
        dynamicmap_rm->SetTwlRwlV2(t_w_l, r_w_l);
        accumed_terrain_mapper->PreProcess(t_w_l, r_w_l);
        instant_terrain_mapper->Run(current_point_cloud, t_w_l, r_w_l, 0);
        accumed_terrain_mapper->Run(current_point_cloud, 0);

        toc(t);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_build(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_in_build = accumed_terrain_mapper->rebuild_point_cloud_n;
    
        dynamicmap_rm->predicted_terrain_height_mat = accumed_terrain_mapper->accumed_predicted_terrain_height_mat.clone();

        /////////////////// test ////////////////////////////
        //dynamicmap_rm->splot[0] = accumed_terrain_mapper->rebuild_point_cloud->size();
        //dynamicmap_rm->splot[1] = accumed_terrain_mapper->rebuild_point_cloud_n->size();

        // Visualizete(true);
        // Visualizeob(true);
        dynamicmap_rm->Addpoint2clouds(cloud_in_build, output_nody_pts);
        //printf("size of rebuild_point_cloud : %ld\n", accumed_terrain_mapper->rebuild_point_cloud->size());
        dynamicmap_rm->PointsInOCV2(accumed_terrain_mapper->rebuild_point_cloud);
        dynamicmap_rm->Addpoint2clouds(dynamicmap_rm->ndy_points, output_nody_pts);

        double dt_this = tic() - t;
        dynamicmap_rm->SetTimes(dt_this);

        
        
    }else{
        dynamicmap_rm->SetTwlRwlV2(t_w_l, r_w_l);
        pcl::PointXYZ po_r;
        Eigen::Vector3d local_point, rotated_local_point;

        /////////////////// test ////////////////////////////
        //dynamicmap_rm->splot[0] = current_point_cloud->size();
        //dynamicmap_rm->splot[1] = 0;
        
        for(size_t i=0; i<current_point_cloud->size(); i++) {
            local_point(0) = current_point_cloud->points[i].x;
            local_point(1) = current_point_cloud->points[i].y;
            local_point(2) = current_point_cloud->points[i].z;

            rotated_local_point = r_w_l * local_point;

            int32_t c = map_cols * dynamicmap_rm->Big_x_bei / 2 + std::floor((rotated_local_point(0) + t_w_l(0) - (dynamicmap_rm->inital_center_x + dynamicmap_rm->inital_residual_x)) / map_resolution);
            int32_t r = map_rows * dynamicmap_rm->Big_y_bei / 2 - 1 - std::floor((rotated_local_point(1) + t_w_l(1) - (dynamicmap_rm->inital_center_y + dynamicmap_rm->inital_residual_y)) / map_resolution);
            if(r<0 || r>=map_rows * dynamicmap_rm->Big_y_bei || c<0 || c>=map_cols * dynamicmap_rm->Big_x_bei) 
            {
            }
            else {
                po_r.x = rotated_local_point(0);
                po_r.y = rotated_local_point(1);
                po_r.z = rotated_local_point(2);
                
                if(dynamicmap_rm->big_terrain_map_mat.at<double>(r,c) == INVALID_HEIGHT){
                    cloud_in_build->points.push_back(po_r);
                    continue;
                }

                CGlobalGridCellV2 *this_cell = dynamicmap_rm->big_global_ob_map2->GetRCLocal(r,c);
                

                double predicted_height = dynamicmap_rm->big_terrain_map_mat.at<double>(r,c);
                float diff = t_w_l(2) - reference_height + rotated_local_point(2) - predicted_height;
                if(diff<0.15) {//0.15
                    cloud_in_build->points.push_back(po_r);
                    this_cell->TerrainObservation();
                }else{
                    cloud_nterrain_build->points.push_back(po_r);
                }   
            }
        }
        dynamicmap_rm->Addpoint2clouds(cloud_in_build, output_nody_pts);
        dynamicmap_rm->PointsInOCV3(cloud_nterrain_build);
        dynamicmap_rm->Addpoint2clouds(dynamicmap_rm->ndy_points, output_nody_pts);
    }

    ////////////////////// test //////////////////////////////
    //dynamicmap_rm->splot[3] = tic();
    //dynamicmap_rm->splot[4] = dynamicmap_rm->dy_points->size();
    //dynamicmap_rm->splot[5] = dynamicmap_rm->ndy_points->size();
    //dynamicmap_rm->Output_splot();

    printf("Remove Dynamic Over ! Run time : ");
    toc(t);
    if(num_cloudin >= MIN_CLOUDS_IN){
        //SetKD(t_w_l);
        dynamicmap_rm->VisualizeDynamicMatkitti(cloud_in_build, LiDARDataPath);
    }
    
}

//////////////////// test ///////////////////
void RemoveDynamicV2::SetStartFrame(int start_frame_){
    start_frame = start_frame_;
}

void RemoveDynamicV2::OutputKD()
{
    std::string load_name = std::string("./oldmap/colored_cloud_ndy.pcd");
    pcl::io::savePCDFileBinary(std::string(load_name), *output_cloud);
    load_name = std::string("./oldmap/colored_cloud_dy.pcd");
    pcl::io::savePCDFileBinary(std::string(load_name), *output_cloud_dy);
}

void RemoveDynamicV2::SetKD(Eigen::Vector3d t_w_l)
{
    pcl::PointXYZRGB pt;
    static bool b_first_runkd = true;
    if(b_first_runkd) {
        for(size_t i=0; i<output_nody_pts->size(); i++){
            pt.x = output_nody_pts->points[i].x + t_w_l(0);
            pt.y = output_nody_pts->points[i].y + t_w_l(1);
            pt.z = output_nody_pts->points[i].z + t_w_l(2);
            int color_idx = (pt.z + 5 - reference_height) * 40;
            if(color_idx<0)
                color_idx = 0;
            if(color_idx>639)
                color_idx = 639;
            pt.r = jet_color_map[color_idx][0];
            pt.g = jet_color_map[color_idx][1];
            pt.b = jet_color_map[color_idx][2];
            output_cloud->push_back(pt);
        }
        for(size_t i=0; i<dynamicmap_rm->dy_points->size(); i++){
            pt.x = dynamicmap_rm->dy_points->points[i].x + t_w_l(0);
            pt.y = dynamicmap_rm->dy_points->points[i].y + t_w_l(1);
            pt.z = dynamicmap_rm->dy_points->points[i].z + t_w_l(2);
            pt.r = 255;pt.g = 10;pt.b = 10;
            output_cloud_dy->push_back(pt);
        }
        b_first_runkd = false;
    }

    for(size_t i=0; i<output_nody_pts->size(); i++){
        pt.x = output_nody_pts->points[i].x + t_w_l(0);
        pt.y = output_nody_pts->points[i].y + t_w_l(1);
        pt.z = output_nody_pts->points[i].z + t_w_l(2);
        int color_idx = (pt.z + 5 - reference_height) * 40;
        if(color_idx<0)
            color_idx = 0;
        if(color_idx>639)
            color_idx = 639;
        pt.r = jet_color_map[color_idx][0];
        pt.g = jet_color_map[color_idx][1];
        pt.b = jet_color_map[color_idx][2];
        output_cloud->push_back(pt);
    }
    for(size_t i=0; i<dynamicmap_rm->dy_points->size(); i++){
        pt.x = dynamicmap_rm->dy_points->points[i].x + t_w_l(0);
        pt.y = dynamicmap_rm->dy_points->points[i].y + t_w_l(1);
        pt.z = dynamicmap_rm->dy_points->points[i].z + t_w_l(2);
        pt.r = 255;pt.g = 10;pt.b = 10;
        output_cloud_dy->push_back(pt);
    }

}

//////////////////// test ///////////////////

void RemoveDynamicV2::EndOut(){
    dynamicmap_rm->OutPutMap(LiDARDataPath);
}

void RemoveDynamicV2::LoadAll()
{
    if(Mode_in == 0){
        dynamicmap_rm->LoadMat();
    }else if(Mode_in == 1){
        dynamicmap_rm->LoadMatV2();
        dynamicmap_rm->LoadBigTerrainMat();
    }else{
        dynamicmap_rm->LoadMatV2();
    }
}

void RemoveDynamicV2::VisualizeDynamicMap(bool b_pause)
{
    if(vis==NULL)
        vis = new pcl::visualization::PCLVisualizer ("DynamicMap");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB pt;

    for(size_t i=0; i<output_nody_pts->size(); i++) {
        pt.x = output_nody_pts->points[i].x;
        pt.y = output_nody_pts->points[i].y;
        pt.z = output_nody_pts->points[i].z;
        pt.r = 0;
        pt.g = 200;
        pt.b = 80;
        colored_cloud->push_back(pt);
    }

    for(size_t i=0; i<dynamicmap_rm->dy_points->size(); i++) {
        pt.x = dynamicmap_rm->dy_points->points[i].x;
        pt.y = dynamicmap_rm->dy_points->points[i].y;
        pt.z = dynamicmap_rm->dy_points->points[i].z;
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        colored_cloud->push_back(pt);
    }
    vis->removeAllPointClouds();
    vis->removeAllShapes();
    vis->addPointCloud (colored_cloud, "cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    if(b_pause)
        vis->spin();
    else
        vis->spinOnce(10);
}

void RemoveDynamicV2::Visualizeob(bool b_pause){
    if(vis==NULL)
        vis = new pcl::visualization::PCLVisualizer ("DynamicMap");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB pt;

    for(size_t i=0; i<accumed_terrain_mapper->rebuild_point_cloud->size(); i++) {
        pt.x = accumed_terrain_mapper->rebuild_point_cloud->points[i].x;
        pt.y = accumed_terrain_mapper->rebuild_point_cloud->points[i].y;
        pt.z = accumed_terrain_mapper->rebuild_point_cloud->points[i].z;
        pt.r = 255;
        pt.g = 255;
        pt.b = 10;
        colored_cloud->push_back(pt);
    }

    vis->removeAllPointClouds();
    vis->addPointCloud (colored_cloud, "cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    vis->spin();
}

void RemoveDynamicV2::Visualizete(bool b_pause){
    if(vis==NULL)
        vis = new pcl::visualization::PCLVisualizer ("DynamicMap");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB pt;

    for(size_t i=0; i<accumed_terrain_mapper->rebuild_point_cloud_n->size(); i++) {
        pt.x = accumed_terrain_mapper->rebuild_point_cloud_n->points[i].x;
        pt.y = accumed_terrain_mapper->rebuild_point_cloud_n->points[i].y;
        pt.z = accumed_terrain_mapper->rebuild_point_cloud_n->points[i].z;
        pt.r = 130;
        pt.g = 130;
        pt.b = 130;
        colored_cloud->push_back(pt);
    }
    
    vis->removeAllPointClouds();
    vis->addPointCloud (colored_cloud, "cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    vis->spin();
}
