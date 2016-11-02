
#include "eigen_cv_geometry_Funs.h"



void Test_RotateVector()
{
    /// eigen

    // yaw pitch( -90~90 ) roll
    Vector3f euler_n2b( -0.5*M_PI,0.4*M_PI,-0.1*M_PI );
    Vector3i eulerOrder( 2,0,1 );

    Matrix3f R_b2n,R_n2b;
    Euler2R(euler_n2b,eulerOrder,R_b2n);
    R_n2b = R_b2n.inverse();

    Quaternionf Q_n2b(R_b2n);
    // get rotate vector from q
    float q_angle_n2b = acos(Q_n2b.w())*2;
    Vector3f qvec_n2b = Q_n2b.vec();
    qvec_n2b.normalize();
//    qvec_n2b = qvec_n2b * q_angle_n2b;

    // get eigen rotate vector
    AngleAxisf angleAxis_n2b(R_b2n);
    Vector3f ei_vec_n2b = angleAxis_n2b.axis() * angleAxis_n2b.angle();

    Matrix3f R_b2n_aAProve = angleAxis_n2b.toRotationMatrix();
    Matrix3f R_n2b_aAProve = R_b2n_aAProve.inverse();

    /// opencv

    cv::Mat C_b2n  = cv::Mat::eye(3,3,CV_32FC1);
    cv::eigen2cv(R_b2n,C_b2n);

    Mat rvec_n2b  = cv::Mat::zeros(3,1,CV_32FC1);
    cv::Rodrigues(C_b2n,rvec_n2b);              // opposite order between in and out

    // get the angle
    float rvec_n2b_angle = norm(rvec_n2b);
    Mat rvec_n2b_normalized;
    cv::normalize(rvec_n2b,rvec_n2b_normalized);

    // prove matrix
    cv::Mat C_n2b_prove,C_b2n_prove;
    cv::Rodrigues(rvec_n2b,C_b2n_prove);        // opposite order between in and out
    C_n2b_prove = C_b2n_prove.inv();




    cout  << " euler_n2b =   "  << euler_n2b.transpose() << endl;
    cout  << " Q_n2b =       "  <<Q_n2b.w() << " ,   "<< Q_n2b.vec().transpose() << endl;
    cout  << " qvec_n2b =    "  <<q_angle_n2b << " ,   "<< qvec_n2b.transpose() << endl;
    cout << endl;

    cout  << " angleAxis_n2b angle , axis = "  << angleAxis_n2b.angle() << " ,   " << angleAxis_n2b.axis().transpose() << endl;
    cout  << " ei_vec_n2b =  "  << ei_vec_n2b.transpose() << endl;
    cout << endl;

    cv::Mat _rvec_n2b, _rvec_n2b_normalized;
    cv::transpose(rvec_n2b,_rvec_n2b);
    cv::transpose(rvec_n2b_normalized,_rvec_n2b_normalized);
    cout  << "rvec_n2b =     "<< _rvec_n2b << endl;
    cout  << "rvec_n2b noangle , normalized =  "<< rvec_n2b_angle << " ,   " <<  _rvec_n2b_normalized << endl;
    cout << endl;

    cout  << " R_n2b =    " << endl << R_n2b << endl;
    cout  << " R_n2b_aAProve =    " << endl << R_n2b_aAProve << endl;
    cout  << " C_n2b_prove =    " << endl << C_n2b_prove << endl;
    cout << endl;

}


void TestCoordinate()
{
    Matrix3f R_b2w ,R_w2b;
    Vector4f euler_b2w(-0.5*M_PI,M_PI,0.5*M_PI,M_PI);
    Vector4i eulerOrder_b2w(0,2,0,1);

    Matrix3f I = Matrix3f::Identity() ;
    R_w2b = AngleAxisf( euler_b2w[0], I.col(eulerOrder_b2w[0]) )*
            AngleAxisf( euler_b2w[1], I.col(eulerOrder_b2w[1]) )*
            AngleAxisf( euler_b2w[2], I.col(eulerOrder_b2w[2]) ) *
            AngleAxisf( euler_b2w[3], I.col(eulerOrder_b2w[3]) ) ;
    R_b2w = R_w2b.inverse();
    cout << "R_b2w= " << endl << R_b2w << endl;
}

// pitch [ -90 ~ 90 ]
// yaw : [ 0~360]  [-180 ~ 180]
// rool : -180 ~ 180
void MyEuler_ZXY( const Eigen::Matrix3f& R_n2b,Eigen::Vector3f& euler_n2b )
{
    float yaw0 = atan2(-R_n2b(1,0) , R_n2b(1,1));
    float pitch0 = asin( R_n2b(1,2) );
    float rool0 = atan2( -R_n2b(0,2),R_n2b(2,2) );

    float yaw = yaw0;
    float pitch = pitch0;
    float rool = rool0;
    // yaw : 0~360
    if( ( yaw0<0 && R_n2b(1,0)<0 ) || ( yaw0>0 && R_n2b(1,0)>0 ) )
    {
        yaw = yaw0 + M_PI;
    }
    if( ( yaw0<0 && R_n2b(1,0)>0 ) )
    {
        yaw = yaw0 + 2*M_PI;
    }

    // rool : -180 ~ 180
    if( rool0<0 && R_n2b(1,3)<0 )
    {
        rool = rool0 + M_PI;
    }
    if( rool0>0 && R_n2b(1,3)>0 )
    {
        rool = rool0 - M_PI;
    }


    euler_n2b << yaw, pitch, rool;
}

void R2Euler( const Eigen::Matrix3f& R_n2b, const Eigen::Vector3i& eulerOrder ,Eigen::Vector3f& euler_n2b)
{
    Matrix3f R_b2n;
    Vector3f euler_b2n;
R_b2n = R_n2b.inverse();
euler_n2b = R_b2n.eulerAngles( eulerOrder[0],eulerOrder[1],eulerOrder[2] );
}


void Euler2R_sameOrder(const Vector3f& euler_n2b,const Vector3i& eulerOrder_n2b, Matrix3f& R_n2b)
{
    Matrix3f R_b2n;
    Euler2R(euler_n2b,eulerOrder_n2b,R_b2n);
    R_n2b = R_b2n.inverse();
}

void Euler2R(const Vector3f& euler_n2b,const Vector3i& eulerOrder_n2b, Matrix3f& R_b2n)
{
    Matrix3f I = Matrix3f::Identity() ;
    R_b2n = AngleAxisf( euler_n2b[0], I.col(eulerOrder_n2b[0]) )*
            AngleAxisf( euler_n2b[1], I.col(eulerOrder_n2b[1]) )*
            AngleAxisf( euler_n2b[2], I.col(eulerOrder_n2b[2]) ) ;
}
