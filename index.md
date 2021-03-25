* **NOTE:** This README.md provides the guidelines in English and Korean (한국어 설명은 **아래에** 있습니다.)

# (Mono) Visual { Inertial | Wheel } Odometry Simualtion
MATLAB simulation of (Mono) visual-inertial odometry (VIO) & visual-wheel odometry

These are MATLAB simulations of (Mono) Visual { Inertial | Wheel } Odometry
These simulations provide the ideal case with **some noises** which can be turned off and on.

* You can use this simulation as you wish with GNU 3.0
* I checked all the demo files whether it can be runned without any amendation, but the MATLAB version can be critical 
* I made this simulations on MATLAB 2019a


## Code Description 

### Caution: Before Using This Simulation
#### Have to set the 'dual_left_monitor' variable to show figures on your monitor
#### Have to select the simulation trajectory with 'trajtype' variable. 

(Each SIMx-files are in a group)
1. **SIM1_w2Dfeat_demo.m** 
> * SIM1 is a simple 2D simulation of camera. Using features with angle and distance constraint.

2. **SIM2_w3Dfeat_demo.m** 
> * SIM2 is a 3D simulation of camera with features and optical flow.
> * SIM2 applies 3D position and attitude in world, body, camera frame.
> * SIM2 selects features with distance, camera heading, intrinsic matrix constraint. 
> * SIM2 visualizes the **camera-robot in 3D** dimension and **projected camera view**
> * SIM2 describes the **Ideal Case** with on-off camera error model (ex. distortion, pixel error, miss tracking)
> * SIM2 saves the results of feature tracking and tracking failure (Variable: LiveTrack, DeadTrack)\

> * **...verSimple.m** is removed.


3. (*not yet*) **SIM3_IMU_demo.m** 
> * ~~SIM3 is a simulation of inverse INS(Traj2IMUdata), INS(IMUdata2Traj) and IMU error model~~ 
4. (*not yet*) **SIM4_WheelOdo_demo.m** 
> * ~~SIM4 is a simulation of inverse WheelOdometry and WheelOdometry with error model~~
5. (*not yet*) **SIM5_Visual_odometry_demo.m** 
> * ~~SIM5 is a simulation of visual odometry using the camera in SIM2 ~~

6. (*Editing*) **SIM6_Visual_inertial_odometry_demo.m** 
> * SIM6 is a simulation of visual-inertial odometry using the camera in SIM2 and ~~IMU in SIM3~~
> * SIM6 visualize as same as SIM2
> * SIM6 calculate epipoar constraint cost to with matched features
> * SIM6 reproduce 3D feature position from recorded tracks

## To Do List
#### Common
- [ ] Trajectory Generation from waypoint or system dynamics
- [x] Adding some trajectory examples
#### SIM2
- [x] Adding N-straking Tracks
- [x] Adding misTracking Ratio
- [ ] Adding noise transfer process
- [ ] Amending the structure to the real situation. (as like just have camera images -> tracking)
- [ ] Amending the notation of the Translation/Rotation/Transformation vectors/Matrixes in the variables.
#### SIM6
- [ ] SIM6 is a simulation of visual-inertial odometry using the camera in SIM2 and IMU in SIM3
- [x] SIM6 visualize as same as SIM2
- [x] SIM6 calculate epipoar constraint cost to with matched features
- [x] SIM6 reproduce 3D feature position from recorded tracks

## Liscence
GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007

*****
###(In Korean)
# (모노)비주얼/관성/휠 오도메트리 시뮬레이션

## 내용 설명 (Guidelines in Korean)
* (모노) 비주얼 { 관성 | 휠 } 오도메트리에 대한 매트랩 시뮬레이션 입니다. 
* 노이즈가 없는 이상적인 케이스가 제공되며, 노이즈는 필요에 따라 사용하거나 사용하지 않도록 설정할 수 있습니다.
* 라이센스는 GNU 버전3를 따릅니다. 이는 자유롭게 사용하되, 같은 라이센스(GNUv3)를 명시하고 그 결과물을 공유해야 한다는 것을 의미합니다.
* 모든 시뮬레이션 파일은 수정없이 바로 동작 가능하도록 체크하고 올립니다만, 문제가 있을 경우 말씀해주세요.
* 혹시 작동이 되지 않는다면, 버전 문제일 수 있습니다. 본 코드는 매트랩2019a에서 작성되었습니다.

## 코드 설명 (Code Description in Korean)

### 주의: 이 시뮬레이션을 사용하기 전에
#### figure를 보기 위해서는 'dual_left_monitor' 변수를 확인하세요
#### 시뮬레이션을 원하는 실험 경로를 'trajtype'에서 확인하세요 

(각각 같은 숫자를 가지는 SIM파일들은 같은 그룹에 속합니다.)
1. **SIM1_w2Dfeat_demo.m** 
> * SIM1은 간단한 2D환경에서의 카메라 시뮬레이션 입니다. 
> * 헤딩과 시야범위의 조건을 통해 간단한 컨셉 코드를 작성하였습니다.

2. **SIM2_w3Dfeat_demo.m** 
> * SIM2는 3D 환경에서의 특징점과 카메라에 대한 시뮬레이션입니다.
> * SIM2에서는 특징점과의 거리, 카메라 방향, 내부 파라미터 조건을 고려해서 특징점을 고릅니다.
> * SIM2에서는 **카메라로봇이 있는 3D환경**과 특징점들이 **카메라에 잡힌 화면**을 보여줍니다.
> * SIM2는 노이즈를 키고 끌 수 있는 이상적인 상황에서의 시뮬레이션입니다. (왜곡, 픽셀 에러, 추적 실패 등)
> * SIM2에서는 feature들을 추적하고 있는 결과를 LiveTrack에 기록하며, 추적이 끝난 결과를 DeadTrack에 (한 스텝 동안만) 저장합니다. (변수 LiveTrack, DeadTrack)

> * **...verSimple.m**은 제거되었습니다.

3. (*예정*) **SIM3_IMU_demo.m** 
> * ~~SIM3는 역 INS와 INS, IMU 에러 모델에 대한 시뮬레이션입니다.~~ 
4. (*예정*) **SIM4_WheelOdo_demo.m** 
> * ~~SIM4는 역 휠 오도메트리와 에러 모델이 고려된 휠 오도메트리에 대한 시뮬레이션입니다.~~
5. (*예정*) **SIM5_Visual_odometry_demo.m** 
> * ~~SIM5는 SIM2의 카메라를 이용한 비주얼 오도메트리에 대한 시뮬레이션입니다.~~

6. (*수정중*) **SIM6_Visual_inertial_odometry_demo.m** 
> * SIM6는 SIM2의 카메라와 SIM3의 IMU를 이용한 비주얼-관성 오도메트리에 대한 시뮬레이션입니다.
> * SIM6는 SIM2와 같은 시각화를 합니다.
> * SIM6는 매칭된 특징점에 대해 에피폴라 조건에 따른 비용함수를 계산합니다.
> * SIM6은 기록된 트랙(LiveTrack or DeadTrack)으로 부터 특징점의 3차원 위치를 복구합니다.

