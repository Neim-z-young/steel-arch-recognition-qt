#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");

    // Setup the cloud pointer
     cloud.reset (new PointCloudT);
     clustered_indices.reset(new pcl::PointIndices);
     remain_indices.reset(new pcl::PointIndices);
     rockface_indices.reset(new pcl::PointIndices);
     steel_arch_cloud.reset(new PointCloudCT);
     colored_cloud.reset(new PointCloudCT);


     // Set up the QVTK window
       viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
       ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
       viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
       ui->qvtkWidget->update ();
//       viewer->addPointCloud (cloud, "cloud");
//       viewer->addPointCloud (colored_cloud, "colored cloud");
       viewer->resetCamera ();
       ui->qvtkWidget->update ();

       //参数设置窗口
       paramDialog = new ParamDialog();
       connect(paramDialog, &ParamDialog::settingSignal, this, &MainWindow::settingParam);
       connect(paramDialog, &ParamDialog::cancelSignal, this, &MainWindow::cancelParam);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete paramDialog;
}


void MainWindow::on_pushButton_clicked()
{
    QString filename;
    filename = QFileDialog::getOpenFileName(this, tr("数据"), "", tr("point cloud data(*.pcd)"));
    if(!filename.isNull())
    {
        if (pcl::io::loadPCDFile<PointT>(filename.toStdString(), *cloud) == -1)
            // load the file
        {
            PCL_ERROR ("Couldn't read file");
            return;
        }
        std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;
        viewer->removeAllPointClouds();
        viewer->removeAllCoordinateSystems();
        viewer->addPointCloud (cloud, "cloud");
        ui->qvtkWidget->update ();

    }
}

void MainWindow::on_pushButton_2_released()
{
    this->hide();
    paramDialog->show();
}

void MainWindow::settingParam(QString p1, QString p2, QString p3, QString p4){
    float arch_steel_gap = p1.toFloat(), arch_steel_thickness = p2.toFloat(),
                scan_size = p3.toFloat(), sensor_resolution = p4.toFloat();
    _PARAM_.reset(new designSpace::TunnelParameter(arch_steel_gap, arch_steel_thickness, scan_size, sensor_resolution));
    //设置新参数后，点云处理也应从头开始
    this->show();
    paramDialog->hide();
}

void MainWindow::cancelParam(){
    this->show();
    paramDialog->hide();
}

void MainWindow::on_pushButton_3_clicked()
{
    //体素滤波
    PointCloudT::Ptr voxelFilterCloud (new PointCloudT);
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);

    voxelGrid.setLeafSize(_PARAM_->VOXEL_SIZE_, _PARAM_->VOXEL_SIZE_, _PARAM_->VOXEL_SIZE_);
    voxelGrid.filter(*voxelFilterCloud);

    std::cout<<"before voxelization size is: "<<(*cloud).points.size()<<std::endl;
    std::cout << "Voxel size is: " << _PARAM_->VOXEL_SIZE_ << std::endl;
    std::cout<<"After voxellization size is: "<<(*voxelFilterCloud).points.size()<<std::endl;

    //聚类参数
    std::cout << "start cluster..." << std::endl;
    designSpace::DBSCAN<PointT> dbscan;
    std::vector<pcl::PointIndices> all_cluster_indices;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    dbscan.setInputCloud(voxelFilterCloud);
    //TODO setIndices()
    dbscan.setRadius(_PARAM_->RADIUS_DBSCAN_);
    dbscan.setTree(tree);
    dbscan.setMinPtsPerCoreObject(_PARAM_->MIN_PTS_);
    dbscan.extract(all_cluster_indices);
    int inx = dbscan.getMaxClusterInx();
    PointCloudCT::Ptr clustered_color_cloud(new PointCloudCT);

    //保存最大聚类点云及其下标并着色
    size_t size = all_cluster_indices[inx].indices.size();
    clustered_indices->indices.resize(size);
    clustered_color_cloud->resize(size);
    int R = random() % 256;
    int G = random() % 256;
    int B = random() % 256;
    for(size_t i=0; i<size; i++){
        clustered_color_cloud->points[i].x = (*voxelFilterCloud).points[all_cluster_indices[inx].indices[i]].x;
        clustered_color_cloud->points[i].y = (*voxelFilterCloud).points[all_cluster_indices[inx].indices[i]].y;
        clustered_color_cloud->points[i].z = (*voxelFilterCloud).points[all_cluster_indices[inx].indices[i]].z;
        clustered_color_cloud->points[i].r = R;
        clustered_color_cloud->points[i].g = G;
        clustered_color_cloud->points[i].b = B;
        clustered_indices->indices[i] = i;
    }
    processed_cloud = clustered_color_cloud;
    colored_cloud = clustered_color_cloud;
    std::cout<<"max cluster size: "<<clustered_color_cloud->points.size()<<std::endl;

    viewer->removeAllPointClouds();
    viewer->addPointCloud (colored_cloud, "colored cloud");
    ui->qvtkWidget->update ();
}

void MainWindow::on_pushButton_4_clicked()
{
    //坐标轴标定
    designSpace::Calibration<pcl::PointXYZRGB> calibration;
    calibration.setInputCloud(processed_cloud);
    calibration.setIndices(clustered_indices);
    calibration.setProjectionGridSize(_PARAM_->ARCH_STEEL_THICKNESS_);
    calibration.setStartAngle(-10.f);
    calibration.setAngleStep(1.f);
    calibration.setEndAngle(10.f);
    calibration.setGroundHeight(_PARAM_->GROUND_HEIGHT_);

    time_t start, end;
    start = time(nullptr);
    std::cout << "start to calibrate: "<<std::endl;
    calibration.setProjectionPlane("yoz");
    calibration.setRotatingAxis('z');
    calibration.calibrate(colored_cloud);
    end = time(nullptr);
    std::cout << "finish to calibrate. time: "<<(end-start)<<" second"<<std::endl;

    processed_cloud = colored_cloud;

    //找质心
    float x = 0, y = 0, z = 0;
    for(const int& inx : clustered_indices->indices){
        x+=processed_cloud->points[inx].x;
        y+=processed_cloud->points[inx].y;
        z+=processed_cloud->points[inx].z;
    }
    x/=processed_cloud->points.size();
    y/=processed_cloud->points.size();
    z/=processed_cloud->points.size();

    std::cout<<"X: "<<x<<" Y: "<<y<<" Z: "<<z<<std::endl;
    viewer->removeCoordinateSystem("ref axes");
    viewer->addCoordinateSystem(10000, x, y, z, "ref axes");
    viewer->removePointCloud("colored cloud");
    viewer->addPointCloud (colored_cloud, "colored cloud");
    ui->qvtkWidget->update ();
}

void MainWindow::on_pushButton_5_clicked()
{
    //移除地面
    designSpace::GroundRemoval<pcl::PointXYZRGB> groundRemoval;
    pcl::PointIndices::Ptr remain_indices_ptr(new pcl::PointIndices);
    groundRemoval.setAxis('z');
    groundRemoval.setGroundHeight(_PARAM_->GROUND_HEIGHT_);
    groundRemoval.setInputCloud(processed_cloud);
    groundRemoval.setIndices(clustered_indices);

    time_t start, end;
    start = time(nullptr);
    std::cout << "start to remove ground: "<<std::endl;
    groundRemoval.remove(*remain_indices_ptr);
    end = time(nullptr);
    std::cout << "finish to remove ground. time: "<<(end-start)<<" second"<<std::endl;

    remain_indices = remain_indices_ptr;
    std::cout<<"remain size: "<<remain_indices_ptr->indices.size()<<std::endl;

    colored_cloud.reset(new PointCloudCT);
    pcl::copyPointCloud(*processed_cloud, remain_indices->indices, *colored_cloud);

    viewer->removePointCloud("colored cloud");
    viewer->addPointCloud (colored_cloud, "colored cloud");
    ui->qvtkWidget->update ();
}

void MainWindow::on_pushButton_6_clicked()
{
    // 参数设置
        designSpace::RockfaceExtraction<pcl::PointXYZRGB> rockfaceExtraction;
        pcl::search::KdTree<PointCT>::Ptr tree(new pcl::search::KdTree<PointCT>);
        rockfaceExtraction.setInputCloud(processed_cloud);
        rockfaceExtraction.setIndices(remain_indices);
        rockfaceExtraction.setTree(tree);
        rockfaceExtraction.setAxis('x');
        rockfaceExtraction.setRadius(_PARAM_->RADIUS_FOR_C_N_);
        rockfaceExtraction.setK(_PARAM_->K_FOR_C_N_);
        rockfaceExtraction.setSegmentLength(_PARAM_->SEGMENT_LENGTH_);

        pcl::PointIndices::Ptr rockface_indices_ptr(new pcl::PointIndices);

        time_t start, end;
        start = time(nullptr);
        std::cout << "start to extract rockface: "<<std::endl;
        int seg_m1, segm2;
        rockfaceExtraction.extract(*rockface_indices_ptr, seg_m1, seg_m2);
        end = time(nullptr);

        std::cout << "finish to extract rockface. time: "<<(end-start)<<" second"<<std::endl;

        //对提取出的岩石表面再次聚类(DBSCAN)
        designSpace::DBSCAN<PointCT> dbscan;
        std::vector<pcl::PointIndices> indices;
        dbscan.setInputCloud(processed_cloud);
        dbscan.setIndices(rockface_indices_ptr);
        dbscan.setTree(tree);
        dbscan.setRadius(_PARAM_->RADIUS_DBSCAN_);
        dbscan.setMinPtsPerCoreObject(_PARAM_->MIN_PTS_);
        dbscan.extract(indices);
        int max_inx = dbscan.getMaxClusterInx();
        rockface_indices_ptr = boost::make_shared<pcl::PointIndices>(indices[max_inx]);

        rockface_indices = rockface_indices_ptr;
        std::cout<<"extracted size: "<<rockface_indices_ptr->indices.size()<<std::endl;

        colored_cloud.reset(new PointCloudCT);
        pcl::copyPointCloud(*processed_cloud, rockface_indices->indices, *colored_cloud);

        //调用viewer->updatePointCloud()会发生很奇怪的错误，应该是vtk的bug
        viewer->removePointCloud("colored cloud");
        viewer->addPointCloud (colored_cloud, "colored cloud");
        ui->qvtkWidget->update ();
}

void MainWindow::on_pushButton_7_clicked()
{
    //参数设置
    designSpace::SteelArchExtraction<PointCT> steelArchExtraction;
    std::cout << "setting parameter..."<<std::endl;
    float x = 0, y = 0, z = 0, min_x = FLT_MAX, max_x = FLT_MIN, min_y = FLT_MAX, max_y = FLT_MIN, min_z = FLT_MAX, max_z = FLT_MIN;
    for (const int& inx:rockface_indices->indices) {
        min_x = fmin(min_x, processed_cloud->points[inx].x);
        min_y = fmin(min_y, processed_cloud->points[inx].y);
        min_z = fmin(min_z, processed_cloud->points[inx].z);

        max_x = fmax(max_x, processed_cloud->points[inx].x);
        max_y = fmax(max_y, processed_cloud->points[inx].y);
        max_z = fmax(max_z, processed_cloud->points[inx].z);

        x += processed_cloud->points[inx].x;
        y += processed_cloud->points[inx].y;
        z += processed_cloud->points[inx].z;
    }
    //重心
    x /= rockface_indices->indices.size();
    y /= rockface_indices->indices.size();
    z /= rockface_indices->indices.size();

    std::cout << "center point is: x:" << x << " y:" << y << " z:" << z << std::endl;

    std::cout << "x axis range is: " <<max_x - min_x<< std::endl;
    std::cout << "y axis range is: " <<max_y - min_y<< std::endl;
    std::cout << "z axis range is: " <<max_z - min_z<< std::endl;

    pcl::search::KdTree<PointCT>::Ptr tree(new pcl::search::KdTree<PointCT>);
    steelArchExtraction.setInputCloud(processed_cloud);
    steelArchExtraction.setIndices(rockface_indices);
    steelArchExtraction.setK(_PARAM_->K_FOR_C_N_);
    steelArchExtraction.setRadius(_PARAM_->RADIUS_FOR_C_N_);
    steelArchExtraction.setTree(tree);
    steelArchExtraction.setArchThickness(_PARAM_->ARCH_STEEL_THICKNESS_);
    steelArchExtraction.setSteelArchGap(_PARAM_->ARCH_STEEL_GAP_);
    steelArchExtraction.setStartArchGap(0.);
    steelArchExtraction.setViewPoint(x, y, z);

    std::vector<PointCT, Eigen::aligned_allocator<PointCT>> steel_arch_points;
    time_t start, end;
    start = time(nullptr);
    std::cout << "start to extract steel arch: "<<std::endl;
    steelArchExtraction.extract(steel_arch_points);
    end = time(nullptr);
    std::cout << "finish to extract steel arch. time: "<<(end-start)<<" second"<<std::endl;

    std::cout << "steel arch point number: "<<steel_arch_points.size()<<std::endl;
    PointCloudCT::Ptr tmp_steel_arch_cloud(new PointCloudCT);
    //提取出最大聚类点云并着色
    int R = 0xea;
    int G = 0x62;
    int B = 0x46;

    for(pcl::PointXYZRGB& point:steel_arch_points){
        point.r = R;
        point.g = G;
        point.b = B;
        tmp_steel_arch_cloud->push_back(point);
    }
    steel_arch_cloud = tmp_steel_arch_cloud;
    viewer->addPointCloud (steel_arch_cloud, "steel arch cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "steel arch cloud");
    ui->qvtkWidget->update ();
}
