## Vins-iOS 修改版

### 修改记录： 
* #### 朱晨曦 优化内存，cpu

* #### 陈作华，陈冬雪修改记录(2017年10月份)

* #### 陈作华修改朱晨曦版本记录：

  - 在VinsViewController.mm里面，第341行缺少OpenCV画点的代码，详情见git内部。


------

#### **以下为2017年10月份修改记录**

> 修改BuildSetting里面的C Language， C++ Language 版本为GNU
>
> VINSVC控制器里，根据index获取朝向，使用getLastKeyframe，不能直接在cpp返回Matrix3d
>
> 修改记录：
>
> - 在VC里，loop_process方法， loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);下面，判断KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);是否为NULL
>
> - 在VC里，process方法，KeyFrame* cur_kf = keyframe_database.getKeyframe(vins.front_pose.cur_index);下面判断获取的kf是否为空。if(cur_kf == NULL) break;
>
> - 在TemplatedDatabase.h里，void TemplatedDatabase<TDescriptor, F>::delete_entry(const EntryId entry_id)方法加上一句    if(m_dBowfile.size() < entry_id) {return;}
> - 在TemplatedLoopDetector.h里，detectLoop方法的最后。修改else if(m_image_keys.size() > entry_id)
>   {
>
>       printf("### 取出赋值");
>       
>       m_image_keys[entry_id] = keys;
>       
>       m_image_descriptors[entry_id] = descriptors;}
>
> - 在TemplatedLoopDetector.h里detectLoop中间，if(m_params.geom_check == GEOM_DI）改成if(m_params.geom_check == GEOM_DI && !&island)，防止island的东西为空。
>
> // 26, 20:35
>
> com.apple.nsurlconnectionloader 线程crash，系统线程。CFNetworking bug
>
> // 26，15：18
>
> wait for imu, only should happen at the beginning
>
> // 26，12：28
>
> loop succ with 322rd image
>
> loop_match before cur 60 60, old 0
>
> loop_match after cur 60 60, old 0
>
> - No loop: Little overlap between this image and the previous one
>
> ---
>
> - No loop: No temporal consistency (k: 1). Best candidate: 97
>
> // 17：55
>
> 2017-09-25 17:36:48.714452+0800 ProductName[1353:378808] GetVinsMatrix == **朝向**
>
> TIMER_marginalization: 22.84ms
>
> release marginlizationinfo
>
> marginalize back
>
> 2017-09-25 17:36:48.730256+0800 ProductName[1353:379333] vins delay 0.660426
>
> loop update visualization
>
> adding feature points 60
>
> parallax sum = 0.792217 parallax_num = 48
>
> number of feature: 100 48
>
> 2017-09-25 17:36:48.747708+0800 ProductName[1353:378808] GetPosition == 498 - -0.007268 - -1.290252 - 1.187085
>
> 2017-09-25 17:36:48.747790+0800 ProductName[1353:378808] GetVinsMatrix == **朝向**
>
> iOS : 1
>
> IOSEvent:LoopBackSuccess(String)
>
> ---
>
> (Filename: /Users/builduser/buildslave/unity/build/artifacts/generated/common/runtime/DebugBindings.gen.cpp Line: 51)
>
> ---
>
> 2017-09-25 17:36:48.749544+0800 ProductName[1353:378891] LoopBackData == **回环**
>
> 获得**keyframe长度219**
>
> VinsData:<LoopBack>m__0()
>
> Loom:RunAction(Object)
>
> ---
>
> (Filename: /Users/builduser/buildslave/unity/build/artifacts/generated/common/runtime/DebugBindings.gen.cpp Line: 51)
>
> ---
>
> solve
>
> wall_time.cc:74 
>
> ---
>
> ComputeStableSchurOrdering
>
> **                                   Delta   Cumulative**
>
> **              CreateHessianGraph :    0.00139      0.00139**
>
> **                     Preordering :    0.00002      0.00141**
>
> **            StableIndependentSet :    0.00054      0.00195**
>
> **         ConstantParameterBlocks :    0.00001      0.00196**
>
> **                           Total :    0.00063      0.00258**
>
> ---
>
> ---
>
> block_sparse_matrix.cc:80 Allocating values array with 174120 bytes.
>
> callbacks.cc:105 iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
>
> **   0  1.460838e+02    0.00e+00    1.05e+03   0.00e+00   0.00e+00  1.00e+04        0    3.53e-03    7.99e-03**
>
> detect_structure.cc:63 Dynamic row block size because the block size changed from 2 to 15
>
> detect_structure.cc:75 Dynamic e block size because the block size changed from 1 to 9
>
> detect_structure.cc:95 Dynamic f block size because the block size changed from 6 to 9
>
> detect_structure.cc:113 Schur complement static structure <-1,-1,-1>.
>
> 2017-09-25 17:36:48.780871+0800 ProductName[1353:378808] GetMatrix3x3 == **朝向 7**
>
> 2017-09-25 17:36:48.781017+0800 ProductName[1353:378808] GetMatrix3x3 == **朝向 17**
>
> 2017-09-25 17:36:48.781059+0800 ProductName[1353:378808] GetMatrix3x3 == **朝向 30**
>
> ProductName was compiled with optimization - stepping may behave oddly; variables may not be available.
>
> 
>
> // 10：43
>
> 2017-09-25 10:43:46.617234+0800 ProductName[941:258559] vins delay 0.670396**
>
> loop update visualization
>
> 2017-09-25 10:43:46.620715+0800 ProductName[941:258292] GetVinsMatrix == **朝向**
>
> iOS : 1
>
> IOSEvent:LoopBackSuccess(String)
>
> ---
>
> (Filename: /Users/builduser/buildslave/unity/build/artifacts/generated/common/runtime/DebugBindings.gen.cpp Line: 51)
>
> ---
>
> 2017-09-25 10:43:46.621619+0800 ProductName[941:258562] LoopBackData == **回环**
>
> 获得**keyframe长度169**
>
> VinsData:<LoopBack>m__0()
>
> Loom:RunAction(Object)
>
> ---
>
> (Filename: /Users/builduser/buildslave/unity/build/artifacts/generated/common/runtime/DebugBindings.gen.cpp Line: 51)
>
> ---
>
> adding feature points 60
>
> parallax sum = 1.548242 parallax_num = 39
>
> number of feature: 106 39
> ProductName was compiled with optimization - stepping may behave oddly; variables may not be available.

