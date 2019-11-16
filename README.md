# DRC-SLAM-Stereo-
Binocular Vision SLAM Algorithm Based on Dynamic Region Culling in Dynamic Environment

Abstract: Visual SLAM (Simultaneous Localization and Mapping) is one of the key technologies in the fields of visual travel aid, augmented reality, and autonomous driving. However, the localization and mapping accuracy of existing SLAM algorithms in dynamic environment usually seriously decreased. To solve this problem, a binocular vision SLAM algorithm based on dynamic region elimination is proposed. Dynamic sparse features are discriminated based on the stereo vision geometric constraints and scene region is segmented according to the scene depth and color information. Combining the results of the above two parts, the dynamic region in the scene is segmented. Furthermore, the ORB features on the dynamic region of the existing binocular ORB-SLAM algorithm are eliminated, so the influence of dynamic scene on the accuracy of SLAM is eliminated. Verified by extensive experiments, the dynamic region segmentation recall rate of the KITTI dataset reaches 92.31%; and the dynamic region segmentation recall rate in the outdoor visual travel aid experiment reaches 93.62%. Compared with binocular ORB-SLAM algorithm, the straight walking localization accuracy is improved by 82.75%, mapping effect is also significantly improved, and the average frame rate of the algorithm reaches 4.6 fps. Results show that the proposed algorithm can significantly improve the localization and mapping accuracy of the binocular vision SLAM algorithm in dynamic scene, and meet the real-time requirement of visual travel aid.

Keywords: Visual SLAM; Localization; Dynamic region; Elimination

Auther:Xu Li, Beihang University
       buaalixu@outlook.com
