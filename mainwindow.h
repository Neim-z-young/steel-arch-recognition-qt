#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//qt
#include <QMainWindow>
#include <QFileDialog>
#include <QDebug>
#include <QtDebug>
#include <qlogging.h>

//jump to dialog
#include "paramdialog.h"

//pcl
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/voxel_grid.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkCamera.h>

//user lib
#include"../../../clion/pclDemo/learning/designLib/tunnelTool.h"
#include"../../../clion/pclDemo/learning/pcdFile/pclDBSCAN.h"
#include"../../../clion/pclDemo/learning/pcdFile/calibratingTool.h"
#include"../../../clion/pclDemo/learning/pcdFile/groundHelper.h"
#include"../../../clion/pclDemo/learning/pcdFile/pclDBSCAN.h"
#include"../../../clion/pclDemo/learning/pcdFile/rockfaceHelper.h"
#include"../../../clion/pclDemo/learning/pcdFile/steelArchHelper.h"
using designSpace::_PARAM_;

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointCT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointCT> PointCloudCT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandler;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    ParamDialog *paramDialog;

protected:
    //show
    pcl::visualization::PCLVisualizer::Ptr viewer;
    PointCloudT::Ptr cloud;
    PointCloudCT::Ptr colored_cloud;
    PointCloudCT::Ptr steel_arch_cloud;
    ColorHandler::Ptr color_handler;
    //processed
    PointCloudCT::Ptr non_calibrated_cloud;
    PointCloudCT::Ptr processed_cloud;
    pcl::PointIndices::Ptr clustered_indices;
    pcl::PointIndices::Ptr remain_indices;
    pcl::PointIndices::Ptr rockface_indices;
    //restored point cloud
    float angle_x;
    float angle_y;
    float angle_z;

private slots:
    void on_pushButton_clicked();
    void on_pushButton_2_released();

    //user slots
    void settingParam(QString, QString, QString, QString);
    void cancelParam();
    void on_pushButton_3_clicked();  //数据预处理
    void on_pushButton_4_clicked();  //坐标轴标定
    void on_pushButton_5_clicked();  //移除地面
    void on_pushButton_6_clicked();  //提取岩石表面
    void on_pushButton_7_clicked();  //钢拱识别
    void on_pushButton_9_clicked();  //标定前点云展示钢拱
    void on_pushButton_10_clicked(); //重置
    void on_pushButton_8_clicked();  //流水线
    void on_pushButton_11_clicked();
    void on_pushButton_12_clicked();
    void on_pushButton_13_clicked();
    void on_pushButton_14_clicked();
};
#endif // MAINWINDOW_H
