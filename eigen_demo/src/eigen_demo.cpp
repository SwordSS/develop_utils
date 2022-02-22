#include <iostream>
#include <Eigen/Eigen>

int main()
{
    //note 矩阵构造
    Eigen::Matrix3d zero = Eigen::Matrix3d::Zero();//零矩阵
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();//单位矩阵
    Eigen::Matrix3d random =  Eigen::Matrix3d::Random();//随机矩阵
    Eigen::MatrixXd mat4d(4,4);//4×4矩阵
    mat4d << 1 , 2, 3, 4,
             5 , 6, 7, 8,
             9 ,10,11,12,
             13,14,15,16;
    Eigen::Vector3d vec3d;//3×1向量
    vec3d << 1,2,3;

    //note 矩阵信息（常规信息）
    int mat4d_rows = mat4d.rows();//行数
    int mat4d_cols = mat4d.cols();//行数
    double sum = mat4d.sum();//所有元素和
    double prod = mat4d.prod();//所有元素乘积
    double mean = mat4d.mean();//所有元素平均值
    double minCoeff = mat4d.minCoeff();//所有元素最小值
    double maxCoeff = mat4d.maxCoeff();//所有元素最大值
    double trace = mat4d.trace();//矩阵的迹

    //note 矩阵信息（常规取值）
    double m11 = mat4d(0,0);//序号0,0的值
    Eigen::MatrixXd mat4d_block = mat4d.block<2,2>(0,0);//从0,0开始取2*2的块
    Eigen::MatrixXd mat4d_row = mat4d.row(0);//取0行
    Eigen::MatrixXd mat4d_col = mat4d.col(0);//取0列
    double v1 = vec3d(0);//序号为0的值

    //note 矩阵信息（运算信息）
    Eigen::Matrix4d mat4d_transpose = mat4d.transpose();//转秩
    Eigen::Matrix4d mat4d_conjugate = mat4d.conjugate();//共厄
    Eigen::Matrix4d mat4d_adjoint = mat4d.adjoint();//伴随
    Eigen::Matrix4d mat4d_inverse = mat4d.inverse();//求逆
    double determinant = mat4d.determinant();//行列式

    //note 除常规的Matrix数据类型外的AngleAxisd，Quaterniond都需要访问其属性进而得到一些基本信息
    //note 旋转向量
    Eigen::AngleAxisd rotation_vector(M_PI/4,Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d rotation_matrix = rotation_vector.matrix();//旋转向量转旋转矩阵
    Eigen::AngleAxisd rotation_vector_(rotation_matrix);//旋转矩阵转旋转向量
    double angle = rotation_vector_.angle()*(180/M_PI);//旋转向量的旋转角(angle输出的是弧度)
    Eigen::MatrixXd axis = rotation_vector_.axis();//旋转向量的旋转轴

    //note 欧拉角
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);//旋转矩阵转欧拉角，ZYX顺序输出欧拉角
    
    //note 四元数
    Eigen::Quaterniond q(1,0,0,0) ;//四元数初始化，输入顺序为w,x,y,z
    Eigen::Quaterniond q_v = Eigen::Quaterniond(rotation_vector);//旋转向量转四元数
    Eigen::Quaterniond q_m = Eigen::Quaterniond(rotation_matrix);//旋转矩阵转四元数
    Eigen::MatrixXd q2m = q.toRotationMatrix();//四元数转旋转矩阵
    Eigen::MatrixXd q_coeffs = q_v.coeffs();//输出顺序为x,y,z,w

    std::cout <<q2m <<std::endl;

    // 两向量求解之间的变换矩阵
    Eigen::Vector3d v_x;
    v_x << 1,0,0; 
    Eigen::AngleAxisd axisd_x2xpi(M_PI/4,Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d R_x2xpi = axisd_x2xpi.matrix();
    Eigen::Vector3d v_x_pi = R_x2xpi* v_x;

    Eigen::Matrix3d R_x2xpi_;
    R_x2xpi_ = Eigen::Quaterniond::FromTwoVectors(v_x, v_x_pi).toRotationMatrix();//四元数换算
    std::cout << "R_x2xpi:" <<std::endl <<R_x2xpi<<std::endl;
    std::cout << "R_x2xpi_:" <<std::endl <<R_x2xpi_<<std::endl;


    return 0;
}