#include "grasping_point_prediction.h"
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))
#include <pcl/visualization/cloud_viewer.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    const std::string points="points";
    const std::string normals="normals";
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud, 0, 255, 0);

    viewer->addPointCloud<pcl::PointXYZI> (cloud, single_color, points,0);
    //viewer->addPointCloudNormals<pcl::PointXYZI>(cloud,1,0.01,"normals", 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, points);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals");
    viewer->addCoordinateSystem (0.1);
    viewer->initCameraParameters ();
    return (viewer);
}


GraspingPointPrediction::GraspingPointPrediction()
{

}

GraspingPointPrediction::GraspingPointPrediction(std::string & file_name, double  mini_box_x_dim, double mini_box_y_dim, double mini_box_z_dim, unsigned int mini_box_x_bins, unsigned int mini_box_y_bins) : 
    _mini_box_x_dim(mini_box_x_dim),
    _mini_box_y_dim(mini_box_y_dim),  // TROQUEI AQUI
    _mini_box_z_dim(mini_box_z_dim),
    _mini_box_x_bins(mini_box_x_bins),
    _mini_box_y_bins(mini_box_y_bins),
    _mini_box_pixel_x_step(_mini_box_x_dim/_mini_box_x_bins),
    _mini_box_pixel_y_step(_mini_box_y_dim/_mini_box_y_bins),
    _mini_box_pixel_x_step_inverse(1.0/_mini_box_pixel_x_step),
    _mini_box_pixel_y_step_inverse(1.0/_mini_box_pixel_y_step),
    _x_offset(_mini_box_x_dim/2.0),
    _y_offset(_mini_box_y_dim/2.0)
{
    _maxPoint[0]=_mini_box_x_dim/2.0;
    _maxPoint[1]=_mini_box_y_dim/2.0;
    _maxPoint[2]=0.01;

    _minPoint[0]=-_mini_box_x_dim/2.0;
    _minPoint[1]=-_mini_box_y_dim/2.0;
    _minPoint[2]=-_mini_box_z_dim;

    std::cout << "_mini_box_x_dim: " << _mini_box_x_dim << std::endl;
    std::cout << "_mini_box_y_dim: " << _mini_box_y_dim << std::endl;
    std::cout << "_mini_box_z_dim: " << _mini_box_z_dim << std::endl;

    std::cout << "_mini_box_x_bins: " << _mini_box_x_bins << std::endl;
    std::cout << "_mini_box_y_bins: " << _mini_box_y_bins << std::endl;

    std::cout << "_mini_box_pixel_x_step: " << _mini_box_pixel_x_step << std::endl;
    std::cout << "_mini_box_pixel_y_step: " << _mini_box_pixel_y_step << std::endl;

    std::cout << "_mini_box_pixel_x_step_inverse: " << _mini_box_pixel_x_step_inverse << std::endl;
    std::cout << "_mini_box_pixel_y_step_inverse: " << _mini_box_pixel_y_step_inverse << std::endl;

    std::cout << "Loading file..." << std::endl;
    _model=svm_load_model(file_name.c_str());
    std::cout << "done" << std::endl;

    _image_grid.resize(_mini_box_x_bins);
    for(std::vector<std::vector<double> >::iterator gridIt=_image_grid.begin();gridIt < _image_grid.end(); ++gridIt)
    {
        gridIt->resize(_mini_box_y_bins);
    }

}


std::vector<double> GraspingPointPrediction::computeGraspingProbability(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> & data)
{

    std::vector<std::vector <double> > features;
    std::vector<double> probabilities;
    // For each grasping point..
    for(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>::iterator dataIt=data.begin(); dataIt < data.end(); ++dataIt)
    {
        // Compute feature...
        features.push_back(computeFeature(*dataIt));
    }

    // Compute features probability
    return predictGraspingPoint(features);
}





std::vector <double> GraspingPointPrediction::computeFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr & data)
{
    ////////////////////////////////
    // Get points inside mini box //
    ////////////////////////////////
    pcl::PointCloud<pcl::PointXYZI>::Ptr data_cropped(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::CropBox<pcl::PointXYZI> cropFilter;
    cropFilter.setInputCloud (data);
    cropFilter.setTranslation(Eigen::Vector3f(0.0,0.0,0.0));
    cropFilter.setRotation(Eigen::Vector3f(0.0,0.0,0.0));
    cropFilter.setMin(_minPoint);
    cropFilter.setMax(_maxPoint);
    cropFilter.filter(*data_cropped);


    /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(data);

    while (!viewer2->wasStopped ())
    {
        viewer2->spinOnce (100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    viewer2  = viewportsVis(data_cropped);

    while (!viewer2->wasStopped ())
    {
        viewer2->spinOnce (100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }*/

    ////////////////////////
    // Compute image grid //
    ////////////////////////

    for(std::vector<std::vector<double> >::iterator gridIt=_image_grid.begin(); gridIt < _image_grid.end(); ++gridIt)
    {
        std::fill(gridIt->begin(),gridIt->end(),-1);
    }

    for(unsigned int i=0; i < data_cropped->size(); ++i)
    {
        unsigned int x_pixel=floor((data_cropped->points[i].x+_x_offset)*_mini_box_pixel_x_step_inverse);
        unsigned int y_pixel=floor((data_cropped->points[i].y+_y_offset)*_mini_box_pixel_y_step_inverse);


        //std::cout << data->points[crop_indices[i]].x << " " << data->points[crop_indices[i]].y << " " << std::endl;
        //std::cout << x_pixel << " " << y_pixel << " " << std::endl;
        //std::cout << crop_indices[i] << std::endl;

        if(data_cropped->points[i].z > _image_grid[x_pixel][y_pixel])
            _image_grid[x_pixel][y_pixel] = data_cropped->points[i].z;
    }

    //////////////////////////////////////
    // Compute partial derivatives grid //
    //////////////////////////////////////

    std::vector<double>  _gradient_grid;

    _gradient_grid.resize( (_mini_box_x_bins-2) * (_mini_box_y_bins-2) );


    unsigned int LINES=_mini_box_y_bins;
    unsigned int COLS=_mini_box_x_bins;

    double max=0.000;

    for(unsigned int i=1; i < LINES-1; ++i)
    {
        for(unsigned int j=1; j < COLS-1; ++j)
        {

            int index = (i-1)*(COLS-2) + j-1;
            _gradient_grid[index] = sqrt( ( (_image_grid[j-1][i]-_image_grid[j+1][i])*(_image_grid[j-1][i]-_image_grid[j+1][i]) + (_image_grid[j][i-1]-_image_grid[j][i+1])*(_image_grid[j][i-1]-_image_grid[j][i+1]) ) );

            if(_gradient_grid[index]>0.6)
                _gradient_grid[index]=0.0;

            if(_gradient_grid[index]>max)
                max=_gradient_grid[index];

        }
    }


    /*IplImage* gradient_image=cvCreateImage(cvSize(COLS-2, LINES-2),IPL_DEPTH_8U,1);

    for(unsigned int i=1; i < LINES-1; ++i)
    {
        for(unsigned int j=1; j < COLS-1; ++j)
        {
            int index = (i-1)*(COLS-2) + j-1;
            CvScalar s;
            s.val[0]=(int)255*(_gradient_grid[index]/max);
            cvSet2D(gradient_image, i-1, j-1 ,s); // set the (i, j) pixel value
        }
    }



    double percent=100*(640/COLS);
    // declare a destination IplImage object with correct size, depth and channels
    IplImage *destination = cvCreateImage( cvSize((int)((gradient_image->width*percent)/100) , (int)((gradient_image->height*percent)/100) ),
            gradient_image->depth, gradient_image->nChannels );



    //use cvResize to resize source to a destination image
    cvResize(gradient_image, destination,CV_INTER_NN );
    cvShowImage("graspability map (OpenCV)", destination );
     cv::waitKey();

    cvReleaseImage(&gradient_image);
    cvReleaseImage(&destination); */
    //std::cout << "FEATURE SIZE:" << _gradient_grid.size() << std::endl;
    return _gradient_grid;
}

std::vector<double> GraspingPointPrediction::predictGraspingPoint(std::vector<std::vector<double> >  & features)
{
    std::vector<double> probabilities;

    //bool predict_probability=false;
    int svm_type=svm_get_svm_type(_model);
    int nr_class=svm_get_nr_class(_model);
    //double *ptr_label;
    //double *ptr_predict_label;
    double *ptr_prob_estimates, *ptr_dec_values;
    //double *ptr_prob_estimates;
    double *prob_estimates=NULL;

    struct svm_node *x;

    int testing_instance_number=features.size(); // Number of features
    int feature_number=features[0].size();
    x = (struct svm_node*)malloc((feature_number+1)*sizeof(struct svm_node) );

    // For each feature
    for(int instance_index=0;instance_index<testing_instance_number;++instance_index)
    {
        int i;
        //double target_label;
        //target_label = ptr_label[instance_index];
        double predict_label;



        // For each feature element
        for(i=0; i < feature_number; ++i)
        {
            x[i].index = i+1;
            x[i].value = features[instance_index][i];
        }
        x[feature_number].index = -1;


        if(svm_type == ONE_CLASS ||
                svm_type == EPSILON_SVR ||
                svm_type == NU_SVR)
        {
            double res;
            predict_label = svm_predict_values(_model, x, &res);
            probabilities.push_back(res);
        }
        else
        {
            double *dec_values = (double *) malloc(sizeof(double) * nr_class*(nr_class-1)/2);
            predict_label = svm_predict_values(_model, x, dec_values);
            if(nr_class == 1)
                probabilities.push_back(1);
            else
                probabilities.push_back(*dec_values);
            free(dec_values);
        }

        //ptr_predict_label[instance_index] = predict_label;
    }

    return probabilities;

}

