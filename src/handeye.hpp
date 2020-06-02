#ifndef HANDEYE_HPP
#define HANDEYE_HPP

#include<iomanip>
#include "parameter.h"
#include "data_class.h"

// #define MAX_AB 100

void HandEye( Pose& res, vector<pair<Pose, Pose>> AB )
{
    Matrix3d Rx, MtM, tmp_c, M = Matrix3d::Zero();
    Vector3d a1, a2, b1, b2, Tx, tmp_d;
    MatrixXd c, d;
    c.resize(3 * AB.size(), 3);
    d.resize(3 * AB.size(), 1);

    if ( AB.size() == 2 )
    {
        Vector3d a1, a2, b1, b2;
        Matrix3d A, B;
        a1 = AB.front().first.getSO3().log();
        a2 = AB.back().first.getSO3().log();
        b1 = AB.front().second.getSO3().log();
        b2 = AB.back().second.getSO3().log();
        A.col(0) = a1;
        A.col(1) = a2;
        A.col(2) = a1.cross(a2);
        B.col(0) = b1;
        B.col(1) = b2;
        B.col(2) = b1.cross(b2);

        Rx = A * B.inverse();
        // cout << a1 << endl;
    }
    else
    {
       for (auto it : AB )
       {
           a1 = it.first.getSO3().log();
           b1 = it.second.getSO3().log();

           M += b1 * a1.transpose();
       }

       MtM = M.transpose() * M;
       EigenSolver<Matrix3d> es(MtM);
       Matrix3d D = es.pseudoEigenvalueMatrix();
	   Matrix3d V = es.pseudoEigenvectors();
       D(0,0) = 1 / sqrt(D(0,0));
       D(1,1) = 1 / sqrt(D(1,1));
       D(2,2) = 1 / sqrt(D(2,2));

    //    cout << D << endl;
       Rx = V * D * V.inverse() * M.transpose();

    //    cout << Rx << endl;
    }

    for (int i = 0; i < AB.size(); i++ )
    {
        tmp_c = Matrix3d::Identity() - AB[i].first.getSO3().matrix();
        tmp_d = AB[i].first.position - Rx * AB[i].second.position;

        c.block<3,3>(3 * i, 0) << tmp_c;
        d.block<3,1>(3 * i, 0) << tmp_d;
    }

    // cout << c << endl;
    // cout << d << endl;
    Tx = ( c.transpose() * c).inverse() * c.transpose() * d;

    res.quat = Rx;
    res.position = Tx; 
}

void getAB ( vector<pair<Pose, Pose>>& AB, vector<Pose>& posev, vector<Pose>& posel )
{
   int sect = posel.size() / MAX_AB ;
   int j = 1;
   auto iter = posel.begin();
   Pose tmp_A, tmp_B, b1, b2;

   for ( int i = 0; i < MAX_AB; i++ )
   {
        // tmp_A = betw_pose(*iter, *(iter + j)); 
        // b1 = find_pose( posev, iter->time );
        // b2 = find_pose( posev, (iter+j)->time );

        // if( b1.time != -1 && b2.time != -1 )
        // {
        //     tmp_B = betw_pose( b1, b2 );
        //     AB.push_back( pair<Pose, Pose> (tmp_A, tmp_B) );
        // }

        // iter += sect;
        // j++;
        // if (j > sect)
        //     j = 1;

        tmp_A = betw_pose( *iter, *(iter + i * sect) );
        b1 = find_pose( posev, iter->time )[0];
        if ( b1.time == -1 )
        {
            iter += sect;
            i++;
            continue;
        }
        b2 = find_pose( posev, (iter + i*sect)->time )[0];
        if ( b2.time != -1 )
        {
            tmp_B = betw_pose( b1, b2 );
            AB.push_back( pair<Pose, Pose> (tmp_A, tmp_B) );
        }
   }
}

#endif