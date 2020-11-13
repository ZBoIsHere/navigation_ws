#include <csc_nav2d_operator/cmd.h>
#include <nav_msgs/GridCells.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

#include <csc_nav2d_navigator/RobotNavigator.h>

#include <set>
#include <map>

#define PI 3.14159265
#define FREQUENCY 10.0

using namespace ros;
using namespace tf;
/**
 * @brief RobotNavigator::RobotNavigator
 * 总的导航算法
 */
RobotNavigator::RobotNavigator()
{	
    //xjh
    mLocalMap = new costmap_2d::Costmap2DROS("global_costmap", mTfListener);//这里相当于开启了一个costmap的node
    mRasterSize = mLocalMap->getCostmap()->getResolution();

    NodeHandle robotNode;

    std::string serviceName;
    robotNode.param("map_service", serviceName, std::string("get_map"));
    mGetMapClient = robotNode.serviceClient<nav_msgs::GetMap>(serviceName);

    mGridMapPublisher = robotNode.advertise<nav_msgs::OccupancyGrid>("globalGridMap",1);
    mLocalGridMapPublisher = robotNode.advertise<nav_msgs::OccupancyGrid>("localGridMap",1);
    mLocalDJGridMapPublisher = robotNode.advertise<nav_msgs::OccupancyGrid>("localDJGridMap",1);
    mCommandPublisher = robotNode.advertise<csc_nav2d_operator::cmd>("cmd", 10);
    mStopServer = robotNode.advertiseService(NAV_STOP_SERVICE, &RobotNavigator::receiveStop, this);
    mPauseServer = robotNode.advertiseService(NAV_PAUSE_SERVICE, &RobotNavigator::receivePause, this);
    mCurrentPlan = NULL;
    mLocalCurrentPlan = NULL;
    mLocalLastPlan = NULL;

    NodeHandle navigatorNode("~/");
    mPlanPublisher = navigatorNode.advertise<sensor_msgs::PointCloud>("globalPlan", 5);
    mLocalPlanPublisher = navigatorNode.advertise<sensor_msgs::PointCloud>("localPlan", 5);

    mMarkerPublisher = navigatorNode.advertise<visualization_msgs::Marker>("markers", 1, true);
    mLocalMarkerPublisher = navigatorNode.advertise<visualization_msgs::Marker>("localMarkers", 1, true);
    mLocalDirectionMarkerPublisher = navigatorNode.advertise<visualization_msgs::Marker>("LocalDirectionMarkers", 1, true);

    // Get parameters
    navigatorNode.param("map_inflation_radius", mInflationRadius, 1.0);
    navigatorNode.param("robot_radius", mRobotRadius, 0.4);
    navigatorNode.param("navigation_goal_distance", mNavigationGoalDistance, 0.2);
    navigatorNode.param("navigation_goal_angle", mNavigationGoalAngle, 0.1);
    navigatorNode.param("navigation_homing_distance", mNavigationHomingDistance, 3.0);
    navigatorNode.param("SpeedMax", mSpeedMax, 0.3);
    navigatorNode.param("SpeedMin", mSpeedMin, 0.3);
    navigatorNode.param("RayTrackSafetyValue", mRayTrackSafetyValue, 0.2);

    mCostObstacle = 100;
    mCostLethal = (1.0 - (mRobotRadius / mInflationRadius)) * (double)mCostObstacle;//阈值，用于判断某个地图点是否是free，必须要求机器人半径小于膨胀半径！
    mCostRayTrack = mCostLethal * mRayTrackSafetyValue;

    robotNode.param("map_frame", mMapFrame, std::string("map"));
    robotNode.param("robot_frame", mRobotFrame, std::string("robot"));
    robotNode.param("odometry_frame", mOdometryFrame, std::string("odometry_base"));//xjh add
    robotNode.param("move_action_topic", mMoveActionTopic, std::string(NAV_MOVE_ACTION));

    // Apply tf_prefix to all used frame-id's
    mRobotFrame = mTfListener.resolve(mRobotFrame);
    mMapFrame = mTfListener.resolve(mMapFrame);
    mOdometryFrame = mTfListener.resolve(mOdometryFrame);

    // Create action servers
    mMoveActionServer = new MoveActionServer(mMoveActionTopic, boost::bind(&RobotNavigator::receiveMoveGoal, this, _1), false);
    mMoveActionServer->start();

    mLocalHasNewMap = false;
    mHasNewMap = false;
    mIsStopped = false;
    mIsPaused = false;
    mIsNavPaused = false;
    mStatus = NAV_ST_IDLE;
    mCellInflationRadius = 0;
    mLocalPlanMissCounter = 0;
    mDebug_show_height = 0.0;
}

RobotNavigator::~RobotNavigator()
{
    delete[] mCurrentPlan;
    delete[] mLocalCurrentPlan;
    delete[] mLocalLastPlan;
    delete mMoveActionServer;
}

/**
 * @brief RobotNavigator::getMap
 * 从地图服务器中获得地图，并计算膨胀半径，但是不对全局地图做膨胀操作
 * @return
 */
bool RobotNavigator::getMap()
{	
    if(mHasNewMap) return true;

    if(!mGetMapClient.isValid())
    {
        ROS_ERROR("GetMap-Client is invalid!");
        return false;
    }

    nav_msgs::GetMap srv;
    if(!mGetMapClient.call(srv))//向ros的mapserver请求地图，这里只需要全局地图
    {
        ROS_INFO("Could not get a map.");
        return false;
    }
    mCurrentMap.update(srv.response.map);

    if(mCurrentPlan) delete[] mCurrentPlan;
    mCurrentPlan = new double[mCurrentMap.getSize()];

    if(mCellInflationRadius == 0)
    {
        ROS_INFO("Navigator is now initialized.");
        mCellInflationRadius = mInflationRadius / mCurrentMap.getResolution();//变成了格子的占用数 1/0.10 = 10格子
        mCellRobotRadius = mRobotRadius / mCurrentMap.getResolution();//变成了格子的占用数
        mInflationTool.computeCaches(mCellInflationRadius);//计算mInflationTool膨胀代价，障碍物距离越远值越小，栅格地图（不是costmap，是概率占用地图），0是free
        mCurrentMap.setLethalCost(mCostLethal);//刷新阈值，用于判断某个地图点是否是free

        mLocalInflationTool.computeCaches(mCellInflationRadius);
    }

    mHasNewMap = true;
    return true;
}

/**
 * @brief RobotNavigator::getLocalMap
 * 基于机器人当前位置，局部costmap，全局mCurrentMap，
 * 在局部costmap 范围 中搜索mCurrentMap中的最小值，如果这个值没有被占用设为目标最小，基于costmap的障碍物和mCurrentMap的障碍物叠加重新计算DJ代价，计算新的局部路径
 * 如果被占用了，则搜索costmap 范围以外的一个mCurrentMap中的最小值，设为目标，基于costmap的障碍物和mCurrentMap的障碍物叠加重新计算DJ代价，计算新的局部路径
 * @param originX
 * @param originY
 * @param widthInCostMap
 * @param heightInCostMap
 * @return 输出 bool mLocalHasNewMap; GridMap mLocalCurrentMap; double* mLocalCurrentPlan;
 */
bool RobotNavigator::getLocalMap(double originX, double originY, unsigned int widthInCostMap, unsigned int heightInCostMap)
{
    if(mLocalHasNewMap) return true;//时间没到 不更新

    mCostmap = mLocalMap->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(mCostmap->getMutex()));
    unsigned int widthInCurrentMap = widthInCostMap * mCostmap->getResolution() /  mCurrentMap.getResolution();
    unsigned int heightInCurrentMap = heightInCostMap * mCostmap->getResolution() /  mCurrentMap.getResolution();

    //要加0.5，否则会有一格的偏移
    unsigned int idX = (originX - mCurrentMap.getOriginX()) / mCurrentMap.getResolution()+0.5;
    unsigned int idY = (originY - mCurrentMap.getOriginY()) / mCurrentMap.getResolution()+0.5;

    //这里有问题 width和height是基于costmap的分辨率计算的 10m距离 全局地图分辨率不同 对应的宽和高也不同

    if(idX+widthInCurrentMap > mCurrentMap.getWidth() || idY+heightInCurrentMap > mCurrentMap.getHeight())
    {
        ROS_ERROR("getLocalMap fail!idX=%d idY=%d w=%d h=%d", idX, idY, widthInCurrentMap, heightInCurrentMap);
        return false;
    }

    nav_msgs::OccupancyGrid tempMap;
    tempMap.data.resize(widthInCurrentMap*heightInCurrentMap);
    tempMap.info.height = heightInCurrentMap;
    tempMap.info.width = widthInCurrentMap;
    tempMap.info.resolution = mCurrentMap.getResolution();
    tempMap.info.origin.position.x = originX;
    tempMap.info.origin.position.y = originY;
    tempMap.info.origin.position.z = mDebug_show_height;

    int id=0;
    char mapValue_temp;
    for(int j=0; j<heightInCurrentMap; j++)
        for(int i=0;i<widthInCurrentMap; i++)
        {
            mapValue_temp = mCurrentMap.getData(idX + i,idY + j);
            if(mapValue_temp>0 && mapValue_temp<mCostObstacle)
                mapValue_temp = 0;
            tempMap.data[id++] = mapValue_temp;
        }

    mLocalCurrentMap.update(tempMap);

    double wx, wy;
    int idLocalX, idLocalY;

    for(unsigned int mx=0; mx<mCostmap->getSizeInCellsX(); mx++)
    {
        for(unsigned int my=0; my<mCostmap->getSizeInCellsY(); my++)
        {
            if(mCostmap->getCost(mx,my) == costmap_2d::LETHAL_OBSTACLE )
            {
                mCostmap->mapToWorld(mx, my, wx, wy);//这个句话是将costmap中的某个点的id坐标转化到meter坐标。全局map坐标系下
                idLocalX = (wx-mLocalCurrentMap.getOriginX())/mLocalCurrentMap.getResolution();
                idLocalY = (wy-mLocalCurrentMap.getOriginY())/mLocalCurrentMap.getResolution();
                mLocalCurrentMap.setData(idLocalX, idLocalY, mCostObstacle);
            }
        }
    }

    if(mLocalCurrentPlan) delete[] mLocalCurrentPlan;
    mLocalCurrentPlan = new double[mLocalCurrentMap.getSize()];

    mLocalCurrentMap.setLethalCost(mCostLethal);//刷新阈值，用于判断某个地图点是否是free
    //mLocalHasNewMap = true;
    return true;
}

/**
 * @brief RobotNavigator::receiveStop
 * 停止标志，可配合手柄
 * @param req
 * @param res
 * @return
 */
bool RobotNavigator::receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    mIsStopped = true;
    res.success = true;
    res.message = "Navigator received stop signal.";
    return true;
}

/**
 * @brief RobotNavigator::receivePause
 * 暂停，按一次暂停导航，再按一次继续导航。
 * @param req
 * @param res
 * @return
 */
bool RobotNavigator::receivePause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{	
    if(mIsPaused)
    {
        mIsPaused = false;
        res.success = false;
        res.message = "Navigator continues.";
    }else
    {
        mIsPaused = true;
        csc_nav2d_operator::cmd stopMsg;
        stopMsg.Turn = 0;
        stopMsg.Velocity = 0;
        mCommandPublisher.publish(stopMsg);
        res.success = true;
        res.message = "Navigator pauses.";
    }
    return true;
}

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

/**
 * @brief RobotNavigator::preparePlan
 * 清除车体位置内的障碍物，更新膨胀后的局部地图。
 * @return
 */
bool RobotNavigator::preparePlan()
{
    // Get the current map
    if(!getMap()) // return false;
    {
        if(mCellInflationRadius == 0) return false;
        ROS_WARN("Could not get a new map, trying to go with the old one...");
    }

    // Where am I?
    if(!setCurrentPosition()) return false;

    // Clear robot footprint in map
    //把机器人在局部地图中的区域设置为free
    unsigned int x = 0, y = 0;
    if(mCurrentMap.getCoordinates(x, y, mStartPoint))
        for(int i = -mCellRobotRadius; i < (int)mCellRobotRadius; i++)
            for(int j = -mCellRobotRadius; j < (int)mCellRobotRadius; j++)
                mCurrentMap.setData(x+i, y+j, 0);

    mInflationTool.inflateMap(&mCurrentMap);//膨胀地图。
    mGridMapPublisher.publish(mCurrentMap.getMap());

    return true;
}

/**
 * @brief RobotNavigator::GridMapWorldToLocalCellCheck
 * 世界栅格坐标转局部栅格坐标检查。
 * @param worldInCellX
 * @param worldInCellY
 * @return
 */
bool RobotNavigator::GridMapWorldToLocalCellCheck(unsigned int worldInCellX, unsigned int worldInCellY)
{
    double worldInMeterX = ((worldInCellX+ 0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginX();
    double worldInMeterY = ((worldInCellY+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginY();

    if(worldInMeterX >= mLocalCurrentMap.getOriginX() + (mLocalCurrentMap.getWidth()-0.5)* mLocalCurrentMap.getResolution() || \
            worldInMeterX <= mLocalCurrentMap.getOriginX() || worldInMeterY <= mLocalCurrentMap.getOriginY() || \
            worldInMeterY >= mLocalCurrentMap.getOriginY() + (mLocalCurrentMap.getHeight()-0.5)* mLocalCurrentMap.getResolution())
    {
        //ROS_WARN("GridMapWorldToLocalCell WorldX=%d, WorldY=%d !LocalX=%d LocalY=%d worldInMeterX=%f worldInMeterY=%f", worldInCellX,  worldInCellY, localInCellX, localInCellY, worldInMeterX, worldInMeterY);
        //ROS_WARN("LocalOriginX=%f LocalOriginY=%f LocalWidth=%f LocalHeight=%f",mLocalCurrentMap.getOriginX(), mLocalCurrentMap.getOriginY(), mLocalCurrentMap.getWidth()* mLocalCurrentMap.getResolution(), mLocalCurrentMap.getHeight()* mLocalCurrentMap.getResolution());
        return false;
    }
    return true;
}

/**
 * @brief RobotNavigator::GridMapWorldToLocalCell
 * 世界栅格坐标转局部栅格坐标
 * 全部转化到物理尺度meter下 cell+0.5后*分辨率。从meter变成cell不用转化
 * @param worldInCellX
 * @param worldInCellY
 * @param localInCellX
 * @param localInCellY
 * @return
 */
bool RobotNavigator::GridMapWorldToLocalCell(unsigned int worldInCellX, unsigned int worldInCellY, unsigned int &localInCellX, unsigned int &localInCellY)
{
    double worldInMeterX = ((worldInCellX+ 0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginX();
    double worldInMeterY = ((worldInCellY+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginY();

    if(worldInMeterX >= mLocalCurrentMap.getOriginX() + (mLocalCurrentMap.getWidth()-0.5)* mLocalCurrentMap.getResolution() || \
            worldInMeterX <= mLocalCurrentMap.getOriginX() || worldInMeterY <= mLocalCurrentMap.getOriginY() || \
            worldInMeterY >= mLocalCurrentMap.getOriginY() + (mLocalCurrentMap.getHeight()-0.5)* mLocalCurrentMap.getResolution())
    {
        ROS_WARN("GridMapWorldToLocalCell WorldX=%d, WorldY=%d !LocalX=%d LocalY=%d worldInMeterX=%f worldInMeterY=%f", worldInCellX,  worldInCellY, localInCellX, localInCellY, worldInMeterX, worldInMeterY);
        ROS_WARN("LocalOriginX=%f LocalOriginY=%f LocalWidth=%f LocalHeight=%f",mLocalCurrentMap.getOriginX(), mLocalCurrentMap.getOriginY(), mLocalCurrentMap.getWidth()* mLocalCurrentMap.getResolution(), mLocalCurrentMap.getHeight()* mLocalCurrentMap.getResolution());
        return false;
    }

    localInCellX = (worldInMeterX - mLocalCurrentMap.getOriginX()) / mLocalCurrentMap.getResolution();
    localInCellY = (worldInMeterY - mLocalCurrentMap.getOriginY()) / mLocalCurrentMap.getResolution();
    //ROS_WARN("!!!mapX=%d, mapY=%d !goalLocalX=%d goalLocalY=%d", worldInCellX,  worldInCellY, localInCellX, localInCellY);
    return true;
}

/**
 * @brief RobotNavigator::
 * 栅格地图，局部坐标转世界坐标
 * @param localInCellX
 * @param localInCellY
 * @param worldInCellX
 * @param worldInCellY
 * @return
 */
bool RobotNavigator::GridMapLocalToWorldCell(unsigned int localInCellX, unsigned int localInCellY, unsigned int &worldInCellX, unsigned int &worldInCellY)
{
    worldInCellX = ((localInCellX+0.5)*mLocalCurrentMap.getResolution() + mLocalCurrentMap.getOriginX() - mCurrentMap.getOriginX())/mLocalCurrentMap.getResolution();
    worldInCellY = ((localInCellY+0.5)*mLocalCurrentMap.getResolution() + mLocalCurrentMap.getOriginY() - mCurrentMap.getOriginY())/mLocalCurrentMap.getResolution();
    return true;
}

/**
 * @brief RobotNavigator::GridMapLocalToWorldMeter
 * 局部物理坐标转世界物理坐标
 * @param localInMeterX
 * @param localInMeterY
 * @param worldInMeterX
 * @param worldInMeterY
 * @return
 */
bool RobotNavigator::GridMapLocalToWorldMeter(double localInMeterX, double localInMeterY, double &worldInMeterX, double &worldInMeterY)
{
    //因为loaclmap的meter的0在图像中心 不在右下角
    worldInMeterX = localInMeterX;// + (mLocalCurrentMap.getOriginX() - mCurrentMap.getOriginX());
    worldInMeterY = localInMeterY;// + (mLocalCurrentMap.getOriginY() - mCurrentMap.getOriginY());
    return true;
}

/**
 * @brief RobotNavigator::GridMapLocalToWorldId
 * 局部栅格id转世界栅格id
 * @param localId
 * @param worldId
 * @return
 */
bool RobotNavigator::GridMapLocalToWorldId(unsigned int localId, unsigned int &worldId)
{
    unsigned int xInWorldCell, yInWorldCell;
    unsigned int xInLocalCell, yInLocalCell;

    mLocalCurrentMap.getCoordinates(xInLocalCell, yInLocalCell, localId);
    if(GridMapLocalToWorldCell(xInLocalCell, yInLocalCell, xInWorldCell, yInWorldCell))
    {
        mCurrentMap.getIndex(xInWorldCell, yInWorldCell, worldId);
        return true;
    }
    else
        return false;

}

/**
 * @brief RobotNavigator::GridMapWorldToLocalId
 * 世界栅格id转局部栅格id
 * @param worldId
 * @param localId
 * @return
 */
bool RobotNavigator::GridMapWorldToLocalId(unsigned int worldId, unsigned int &localId)
{
    unsigned int xInWorldCell, yInWorldCell;
    unsigned int xInLocalCell, yInLocalCell;

    mCurrentMap.getCoordinates(xInWorldCell, yInWorldCell, worldId);
    if(GridMapWorldToLocalCell(xInWorldCell, yInWorldCell, xInLocalCell, yInLocalCell))
    {
        mLocalCurrentMap.getIndex(xInLocalCell, yInLocalCell, localId);
        return true;
    }
    else
    {
        return false;
    }
}


/**
 * @brief RobotNavigator::createLocalMap
 * 局部规划使用1.5倍大小的局部地图，其中叠加1倍大小的局部障碍物，换言之 靠边上的0.5倍的区域不存在局部障碍物，只存在全局障碍物 用于导航规划。
 * @return
 */
bool RobotNavigator::createLocalMap()
{
    //必须要加这两句话 否则会返回空的costmap！
    mCostmap = mLocalMap->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(mCostmap->getMutex()));

    //注意 这个是costmap中的cell 他和mapserver发布的分辨率会不同
    unsigned int localMapWidthInCell =  (mCostmap->getSizeInMetersX() + mRobotRadius*2 + 0.2) / mCostmap->getResolution();
    unsigned int localMapHeightInCell = (mCostmap->getSizeInMetersY() + mRobotRadius*2 + 0.2) / mCostmap->getResolution();

    double localMapStartInMeterX = mCurrentPositionX - (localMapWidthInCell)*mCostmap->getResolution() / 2.0;
    double localMapStartInMeterY = mCurrentPositionY - (localMapHeightInCell)*mCostmap->getResolution() / 2.0;

    if(!getLocalMap(localMapStartInMeterX, localMapStartInMeterY, localMapWidthInCell, localMapHeightInCell))
    {
        ROS_WARN("Could not get a new local map, trying to go with the old one...");
        return false;//这个应该是不会发生的
    }
    //ROS_INFO("mCurrentPositionX=%f, mCurrentPositionY=%f, MeterX=%f , MeterY=%f , WidthInCell=%d, HeightInCell=%d",mCurrentPositionX,mCurrentPositionY,localMapStartInMeterX, localMapStartInMeterY, localMapWidthInCell, localMapHeightInCell);


    //实际上因为全局定位会有偏差，导致在狭窄的地方，机器人被定位到了全局map的障碍物点上，导致无法找到局部goal。
    // Clear robot footprint in map
    //把机器人在局部地图中的区域设置为free
    unsigned int x = 0, y = 0;
    if(mLocalCurrentMap.getCoordinates(x, y, mLocalStartPoint))
        for(int i = -mCellRobotRadius; i < (int)mCellRobotRadius; i++)
            for(int j = -mCellRobotRadius; j < (int)mCellRobotRadius; j++)
                mLocalCurrentMap.setData(x+i, y+j, 0);

    mLocalInflationTool.inflateMap(&mLocalCurrentMap);//膨胀地图。
    mLocalGridMapPublisher.publish(mLocalCurrentMap.getMap());

    return true;
}


/**
 * @brief RobotNavigator::createLocalGoal
 * 在创建全局地图时会清除机器人区域，然后做全局plan计算
 * 当机器人要获得localplan时要根据全局plan找到localGoal
 * 但是由于全局plan只在初始化时会清除机器人区域，之后不会在地图中清除对应的区域，因此在做局部规划时，将当前机器人放进去算会导致在计算局部goal时卡死，周围没有空闲区域。
 * 因为全局定位难免有偏差，这个问题会出现在特别狭窄的地方。
 * @return
 */
bool RobotNavigator::createLocalGoal()
{
    //要查找局部地图中的goal点，基于已知的全局mCurrentPlan
    unsigned int index = mStartPoint;
    std::vector<std::pair<double, double> > points;

    //因为mCurrentPlan是一开始规划时就确定的，有可能当前位置开到了mCurrentPlan中值为-1的无效地方导致无法查找到局部goal
    //因为地图是统一膨胀的，再加上地图并不标的那么精确，车子在靠近路边缘时会进入mCurrentPlan中值为-1的无效地方。
    //这里查找车子以及周围1m内的空闲点，同时存在mCurrentPlan值的点（不是-1），找一个最近点作为车子的起始点进行规划
    if(mCurrentPlan[index] == -1)//如果初始位置不空闲，就需要修改初始位置
    {
        unsigned int start_x, start_y;
        mCurrentMap.getCoordinates(start_x, start_y, index);
        int RSize = 1.0/mCurrentMap.getResolution();
        std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(index, RSize);//获得周围8个领域
        int lenth = 100;
        for(int i=0; i<neighbors.size(); i++)
        {
            if(mCurrentPlan[neighbors[i]] != -1)
            {
                unsigned int x = 0, y = 0;
                mCurrentMap.getCoordinates(x, y, neighbors[i]);
                int lenth_temp = abs((int)start_x - (int)x) + abs((int)start_y - (int)y);
                if(lenth_temp < lenth)
                {
                    lenth = lenth_temp;
                    index = neighbors[i];
                }
            }
        }
        if(lenth == 100)//在机器人为中心的一米内也没找到空闲点
        {
            ROS_ERROR("createLocalGoal Fail! Can not find a free start point ! mStartPoint = %d", mStartPoint);
            return false;
        }
        ROS_WARN("mStartPoint %d is not free, adjust to point %d", mStartPoint, index);
        mStartPoint = index;//在查找局部goal时就修改掉当前位置，后面做局部路径规划时就不用再次查找起始点了。
        setLocalCurrentPosition();
    }

    while(true)
    {
        unsigned int x = 0, y = 0;
        if(mCurrentMap.getCoordinates(x,y,index))
            points.push_back(std::pair<double, double>(
                                 ((x+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginX(),
                                 ((y+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginY()
                                 ));

        //将map中的点转化到局部地图中
        //必须先检查这个边界条件！
        unsigned int worldInCellX, worldInCellY;
        mCurrentMap.getCoordinates(worldInCellX, worldInCellY, index);
        unsigned int localInCellX, localInCellY;
        if(!GridMapWorldToLocalCellCheck(worldInCellX, worldInCellY))
        {
            points.pop_back();//删除最后一个点，保证所有点都在局部地图中
            break;//超出局部地图就停止搜索。但是最后一个点已经被保存了，刚好处于局部地图的外面
        }

        if(mCurrentPlan[index] == 0)
        {
            //ROS_INFO("Find value 0 index = %d",index);
            break;//迭代到了goal
        }

        unsigned int next_index = index;
        std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(index);
        for(unsigned int i = 0; i < neighbors.size(); i++)//查找8个领域里距离最小的值，也就是最大的梯度方向
        {
            if(mCurrentPlan[neighbors[i]] >= 0 && mCurrentPlan[neighbors[i]] < mCurrentPlan[next_index])
                next_index = neighbors[i];
        }
        if(index == next_index)
        {
            ROS_INFO("index == next_index");
            ROS_INFO("mCurrentPlan[next_index]=%f", mCurrentPlan[next_index]);
            break;//当周围没有空闲区域时
        }
        index = next_index;
    }
    /*
  //这里是全局地图坐标下的点
  sensor_msgs::PointCloud plan_msg;
  plan_msg.header.frame_id = mMapFrame.c_str();
  plan_msg.header.stamp = Time::now();

  sensor_msgs::ChannelFloat32 temp;
  geometry_msgs::Point32 temp2;
  temp.name = "intensity";

  for(unsigned int i = 0; i < points.size(); i++)
  {
    temp2.x = points[i].first;
    temp2.y = points[i].second;
    temp2.z = 0;
    plan_msg.points.push_back(temp2);
    temp.values.push_back(i);
  }
  plan_msg.channels.push_back(temp);
  mPlanPublisher.publish(plan_msg);
  */
    //std::cout << "find local goal complete.plan size = "<< points.size() <<std::endl;

    double goalWorldX, goalWorldY;
    if(points.size()>0)
    {
        goalWorldX = points.back().first;
        goalWorldY = points.back().second;
        //ROS_INFO("goalWorld size=%d", points.size());
    }
    else
    {
        goalWorldX = points[0].first;
        goalWorldY = points[0].second;
        //这样写有可能内存溢出，后期要修改，局部找路径找不到的情况。
        ROS_ERROR("goalWorld size is 0");
        return false;
    }

    unsigned int goalLocalX, goalLocalY;

    //换算到grid坐标系
    int mapX =  (double)(goalWorldX - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
    int mapY =  (double)(goalWorldY - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
    if(mapX < 0) mapX = 0;
    if(mapX >= (int)mCurrentMap.getWidth()) mapX = mCurrentMap.getWidth() - 1;
    if(mapY < 0) mapY = 0;
    if(mapY >= (int)mCurrentMap.getHeight()) mapY = mCurrentMap.getHeight() - 1;

    GridMapWorldToLocalCell(mapX, mapY, goalLocalX, goalLocalY);

    //ROS_INFO("reday to LocalPlan goalMapX=%d, goalMmapY=%d !goalLocalX=%d goalLocalY=%d", mapX, mapY, goalLocalX, goalLocalY);
    if(mLocalCurrentMap.getIndex(goalLocalX, goalLocalY, mLocalGoalPoint))//获得goal的id号，用于下一步规划
    {
        return true;
    }
    else
    {
        ROS_ERROR("Couldn't convert mLocalGoalPoint goalLocalX=%d goalLocalY=%d mLocalGoalPoint=%d", goalLocalX, goalLocalY, mLocalGoalPoint);
        return false;
    }
}

/**
 * @brief RobotNavigator::createPlan
 * 计算全局路径
 * @return
 */
bool RobotNavigator::createPlan()
{	
    ROS_DEBUG("Map-Value of goal point is %d, lethal threshold is %d.", mCurrentMap.getData(mGoalPoint), mCostLethal);

    unsigned int goal_x = 0, goal_y = 0;
    if(mCurrentMap.getCoordinates(goal_x,goal_y,mGoalPoint))
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mCurrentMap.getOriginX() + (((double)goal_x+0.5) * mCurrentMap.getResolution());
        marker.pose.position.y = mCurrentMap.getOriginY() + (((double)goal_y+0.5) * mCurrentMap.getResolution());
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = mCurrentMap.getResolution() * 1.0;
        marker.scale.y = mCurrentMap.getResolution() * 1.0;
        marker.scale.z = 3.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        mMarkerPublisher.publish(marker);
    }else
    {
        ROS_ERROR("Couldn't ressolve goal point coordinates!");
    }

    Queue queue;

    // Reset the plan
    int mapSize = mCurrentMap.getSize();
    for(int i = 0; i < mapSize; i++)
    {
        mCurrentPlan[i] = -1;
    }

    if(mCurrentMap.isFree(mGoalPoint))
    {
        queue.insert(Entry(0.0, mGoalPoint));
        mCurrentPlan[mGoalPoint] = 0;
    }else
    {
        // Initialize the queue with area around the goal point
        int reach = mCellRobotRadius + (1.0 / mCurrentMap.getResolution());
        std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(mGoalPoint, reach);
        for(unsigned int i = 0; i < neighbors.size(); i++)
        {
            queue.insert(Entry(0.0, neighbors[i]));
            mCurrentPlan[neighbors[i]] = 0;
        }
        //如果goal被占用，则寻找周围车身半径+1m范围的区域内所有的空闲区域，都设置为0的代价。
    }

    Queue::iterator next;
    double distance;
    unsigned int x, y, index;
    double linear = mCurrentMap.getResolution();
    double diagonal = std::sqrt(2.0) * linear;

    // Do full search with Dijkstra-Algorithm
    while(!queue.empty())
    {
        // Get the nearest cell from the queue
        next = queue.begin();
        distance = next->first;
        index = next->second;
        queue.erase(next);

        if(mCurrentPlan[index] >= 0 && mCurrentPlan[index] < distance) continue;
        //当地图中的点已经有代价，并且这个点代价比distance小（因为被其他相邻节点更新了），就不需要再更新这个节点。
        //注释掉这个break是为了防止车子在导航中途因为避障碍，开向其他地方后无法再找到周围的迭代方向
        //这里是全局规划，反正只做一次，就慢慢的规划好了。
        //		if(index == mStartPoint) break;

        // Add all adjacent cells
        if(!mCurrentMap.getCoordinates(x, y, index)) continue;
        std::vector<unsigned int> ind;
        ind = mCurrentMap.getNeighbors(index, true);
        for(unsigned int it = 0; it < ind.size(); it++)
        {
            unsigned int i = ind[it];
            if(mCurrentMap.isFree(i))
            {
                double delta = (it < 4) ? linear : diagonal;
                double newDistance = distance + delta + \
                        (10 * delta * (double)mCurrentMap.getData(i) / (double)mCostObstacle);//最后一项，在基本的DJ算法上，增加cost的代价。(double)mCurrentMap.getData(i) / (double)mCostObstacle是0-1的值，所以放大10倍
                if(mCurrentPlan[i] == -1 || newDistance < mCurrentPlan[i])
                {
                    queue.insert(Entry(newDistance, i));
                    mCurrentPlan[i] = newDistance;
                }
            }
        }
    }

    if(mCurrentPlan[mStartPoint] < 0)
    {
        ROS_ERROR("In createPlan. No way between robot and goal! mStartPoint = %d", mStartPoint);
        return false;
    }

    //以上是将整张地图根据目标点进行cost计算，相当暴力和耗时的方法，没有任何优化
    //之后再是基于这张costmap进行路径规划
    publishPlan();
    return true;
}

/**
 * @brief RobotNavigator::createLocalPlan
 * 计算局部路径
 * @return
 */
bool RobotNavigator::createLocalPlan()
{
    unsigned int goal_x = 0, goal_y = 0;
    if(mLocalCurrentMap.getCoordinates(goal_x,goal_y,mLocalGoalPoint))//传进来的就是局部坐标系下的点
    {
        unsigned int world_goal_x, world_goal_y;
        GridMapLocalToWorldCell(goal_x, goal_y, world_goal_x, world_goal_y);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mCurrentMap.getOriginX() + (((double)world_goal_x) * mCurrentMap.getResolution());
        marker.pose.position.y = mCurrentMap.getOriginY() + (((double)world_goal_y) * mCurrentMap.getResolution());
        marker.pose.position.z = mDebug_show_height;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = mCurrentMap.getResolution() * 1.0;
        marker.scale.y = mCurrentMap.getResolution() * 1.0;
        marker.scale.z = 3.0;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        mLocalMarkerPublisher.publish(marker);
    }else
    {
        ROS_ERROR("Couldn't ressolve goal point coordinates!");
        return false;
    }

    Queue queue;

    // Reset the plan
    int mapSize = mLocalCurrentMap.getSize();
    for(int i = 0; i < mapSize; i++)
    {
        mLocalCurrentPlan[i] = -1;
    }
    //ROS_INFO("mLocalGoalPoint=%d, goal_x=%d, goal_y=%d", mLocalGoalPoint, goal_x, goal_y);

    if(mLocalCurrentMap.isFree(mLocalGoalPoint))
    {
        queue.insert(Entry(0.0, mLocalGoalPoint));
        mLocalCurrentPlan[mLocalGoalPoint] = 0;
    }else
    {
        //在局部地图中，这个条件一般情况不会成立
        //因为全局规划路线时就不会将被占用的点当作路径点
        //但是当靠近goal，goal处于局部costmap区域中（mLocalCurrentMap是1.5倍的costmap大小）就会出现这个问题
        //如果目标点，以及周围1m内都被障碍物占用了，这里的写法是有bug的。
        // Initialize the queue with area around the goal point
        int reach = mCellRobotRadius + (1.0 / mLocalCurrentMap.getResolution());
        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(mLocalGoalPoint, reach);
        for(unsigned int i = 0; i < neighbors.size(); i++)
        {
            queue.insert(Entry(0.0, neighbors[i]));
            mLocalCurrentPlan[neighbors[i]] = 0;
        }
        ROS_WARN("local goal is not free! find free neighbors number %d, reach=%d", (int)neighbors.size(), reach);
        //如果goal被占用，则寻找周围车身半径+1m范围的区域内所有的空闲区域，都设置为0的代价。
    }
    //将上一次的最短路径转化到当前map坐标下。
    std::vector<unsigned int> lastLocalPlanInd;
    for(unsigned int i=0; i<mLocalPlanPointsInWorldCell.size(); i++)
    {
        unsigned int lastLocalPlanId;
        if(GridMapWorldToLocalId(mLocalPlanPointsInWorldCell[i], lastLocalPlanId))
        {
            lastLocalPlanInd.push_back(lastLocalPlanId);
        }
    }

    Queue::iterator next;
    double distance;
    unsigned int x, y, index;
    unsigned int start_x=0, start_y=0;
    if(!mCurrentMap.getCoordinates(start_x, start_y, mLocalStartPoint)) return false;

    double linear = mLocalCurrentMap.getResolution();
    double diagonal = std::sqrt(2.0) * linear;

    // Do full search with Dijkstra-Algorithm
    while(!queue.empty())
    {
        // Get the nearest cell from the queue
        next = queue.begin();
        distance = next->first;
        index = next->second;
        queue.erase(next);

        //当地图中的点已经有代价，并且这个点代价比distance小（因为被其他相邻节点更新了），就不需要再更新这个节点。
        //if(mLocalCurrentPlan[index] >= 0 && mLocalCurrentPlan[index] < distance) continue;

        //注释掉这个break是为了防止车子在导航中途因为避障碍，开向其他地方后无法再找到周围的最优迭代方向
        //个人觉得在局部地图中规划时，这个break可以加
        if(index == mLocalStartPoint) break;

        // Add all adjacent cells
        if(!mLocalCurrentMap.getCoordinates(x, y, index)) continue;

        std::vector<unsigned int> ind;
        ind = mLocalCurrentMap.getNeighbors(index, true);

        for(unsigned int it = 0; it < ind.size(); it++)
        {
            unsigned int i = ind[it];
            if(mLocalCurrentMap.isFree(i))
            {
                double delta = (it < 4) ? linear : diagonal;
                //增加上一帧的最优路径奖励
                auto itt = find(lastLocalPlanInd.begin(),lastLocalPlanInd.end(), i);
                if (itt != lastLocalPlanInd.end())
                {
                    //vec中存在value值
                    delta = delta;
                }
                else
                {
                    //vec中不存在value值
                    delta = 1.5*delta;
                }

                double newDistance = mLocalCurrentPlan[index] + delta + \
                        (10 * delta * (double)mLocalCurrentMap.getData(i) / (double)mCostObstacle);//最后一项，在基本的DJ算法上，增加cost的代价。(double)mCurrentMap.getData(i) / (double)mCostObstacle是0-1的值，所以放大10倍

                if(mLocalCurrentPlan[i] == -1 || newDistance < mLocalCurrentPlan[i])
                {
                    if(!mLocalCurrentMap.getCoordinates(x, y, i)) continue;

                    // 增加启发函数
                    int dx = std::abs((int)x-(int)start_x);
                    int dy = std::abs((int)y-(int)start_y);
                    double heuristic = linear*(dx + dy) + (diagonal - 2*linear) * std::min(dx, dy);

                    double priority = newDistance + heuristic;

                    queue.insert(Entry(priority, i));
                    mLocalCurrentPlan[i] = newDistance;
                }
            }
        }
    }
    //DJ结束以后检查初始位置是否是可行的
    if(mLocalCurrentPlan[mLocalStartPoint] < 0)
    {
        ROS_WARN("In createLocalPlan. No way between robot and goal!");
        return false;
    }
    return true;
}

/**
 * @brief RobotNavigator::publishPlan
 * 发布全局路径
 */
void RobotNavigator::publishPlan()
{

    unsigned int index = mStartPoint;
    std::vector<std::pair<double, double> > points;
    while(true)
    {
        unsigned int x = 0, y = 0;
        if(mCurrentMap.getCoordinates(x,y,index))
            points.push_back(std::pair<double, double>(
                                 ((x+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginX(),
                                 ((y+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginY()
                                 ));

        if(mCurrentPlan[index] == 0) break;//迭代到了goal

        unsigned int next_index = index;

        std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(index);//获得周围8个领域
        for(unsigned int i = 0; i < neighbors.size(); i++)//查找8个领域里距离最小的值，也就是最大的梯度方向
        {
            if(mCurrentPlan[neighbors[i]] >= 0 && mCurrentPlan[neighbors[i]] < mCurrentPlan[next_index])
                next_index = neighbors[i];
        }

        if(index == next_index) break;//这个条件，暂时想不出来有什么可能会落入
        index = next_index;
    }
    //这里是全局地图坐标下的点
    sensor_msgs::PointCloud plan_msg;
    plan_msg.header.frame_id = mMapFrame.c_str();
    plan_msg.header.stamp = Time::now();

    sensor_msgs::ChannelFloat32 temp;
    geometry_msgs::Point32 temp2;
    temp.name = "intensity";

    for(unsigned int i = 0; i < points.size(); i++)
    {
        temp2.x = points[i].first;
        temp2.y = points[i].second;
        temp2.z = 0;
        plan_msg.points.push_back(temp2);
        temp.values.push_back(i);
    }
    plan_msg.channels.push_back(temp);
    mPlanPublisher.publish(plan_msg);
}

/**
 * @brief RobotNavigator::publishLocalPlan
 * 发布局部路径
 *
 */
void RobotNavigator::publishLocalPlan()
{
    unsigned int index = mLocalStartPoint;
    unsigned int indexInWorld;
    std::vector<std::pair<double, double> > points;
    mLocalPlanPointsInWorldCell.clear();
    GridMapLocalToWorldId(index, indexInWorld);
    mLocalPlanPointsInWorldCell.push_back(indexInWorld);
    while(true)
    {
        unsigned int x = 0, y = 0;
        if(mLocalCurrentMap.getCoordinates(x,y,index))
            points.push_back(std::pair<double, double>(
                                 ((x+0.5) * mLocalCurrentMap.getResolution()) + mLocalCurrentMap.getOriginX(),
                                 ((y+0.5) * mLocalCurrentMap.getResolution()) + mLocalCurrentMap.getOriginY()
                                 ));

        if(mLocalCurrentPlan[index] == 0) break;//迭代到了goal

        unsigned int next_index = index;

        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(index);//获得周围8个领域
        for(unsigned int i = 0; i < neighbors.size(); i++)//查找8个领域里距离最小的值，也就是最大的梯度方向
        {
            if(mLocalCurrentPlan[neighbors[i]] >= 0 && mLocalCurrentPlan[neighbors[i]] < mLocalCurrentPlan[next_index])
                next_index = neighbors[i];
        }

        if(index == next_index) break;//这个条件，暂时想不出来有什么可能会落入
        index = next_index;

        GridMapLocalToWorldId(index, indexInWorld);
        mLocalPlanPointsInWorldCell.push_back(indexInWorld);
    }
    //这里是局部地图坐标下的点要转化到全局坐标进行发布

    sensor_msgs::PointCloud plan_msg;
    plan_msg.header.frame_id = mMapFrame.c_str();
    plan_msg.header.stamp = Time::now();

    sensor_msgs::ChannelFloat32 temp;
    geometry_msgs::Point32 temp2;
    temp.name = "intensity";

    double wx, wy;
    for(unsigned int i = 0; i < points.size(); i++)
    {
        GridMapLocalToWorldMeter(points[i].first, points[i].second, wx, wy);
        temp2.x = wx;
        temp2.y = wy;
        temp2.z = mDebug_show_height;
        plan_msg.points.push_back(temp2);
        temp.values.push_back(i);
    }
    plan_msg.channels.push_back(temp);
    mLocalPlanPublisher.publish(plan_msg);
}

/**
 * @brief RobotNavigator::RayTrackisClean
 * 射线清除法，优化局部目标点
 * 在先前的基础上 基于DJST计算出来的路径 从近处开始向远处查询，
 * 计算mLocalStartPoint到第i个路径点连成的直线，遍历这个直线上所有的栅格，如果大于某一个占用阈值，就认为失败否则就是可行点
 * @param endPointIndex
 * @return
 */
bool RobotNavigator::RayTrackisClean(unsigned int endPointIndex)
{
    if(endPointIndex == mLocalStartPoint)
        return true;

    unsigned int startX = 0, startY = 0;
    unsigned int endX = 0, endY = 0;

    mLocalCurrentMap.getCoordinates(startX, startY, mLocalStartPoint);
    mLocalCurrentMap.getCoordinates(endX, endY, endPointIndex);

    int deltaX = abs(int(startX)-int(endX));
    int deltaY = abs(int(startY)-int(endY));
    //ROS_INFO("startX%d, startY%d endX%d, endY%d deltaX%d deltaY %d",startX, startY,endX, endY,deltaX,deltaY);
    //因为一开始排除了同一个点的情况 所以deltaX deltaY 不可能同时为0 ，所以除数不会为0
    if(deltaX > deltaY)
    {
        double stepX = (int(endX)-int(startX)) * mLocalCurrentMap.getResolution() / deltaX;
        double stepY = (int(endY)-int(startY)) * mLocalCurrentMap.getResolution() / deltaX;
        //ROS_INFO("stepX%f stepY%f", stepX, stepY);

        for(int i=1; i<= deltaX; i++)
        {
            double meterX = startX*mLocalCurrentMap.getResolution() + i*stepX;//为了统一格式。。虽然我知道这么写浪费计算资源
            double meterY = startY*mLocalCurrentMap.getResolution() + i*stepY;

            int cellX = meterX/mLocalCurrentMap.getResolution();
            int cellY = meterY/mLocalCurrentMap.getResolution();
            //ROS_INFO("cellX %d  cellY %d, mCostRayTrack%d cell%d startCell%d ", cellX, cellY, mCostRayTrack, mLocalCurrentMap.getData(cellX, cellY), mLocalCurrentMap.getData(startX, startY));
            if(mLocalCurrentMap.getData(cellX, cellY) > mCostRayTrack)
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        double stepX = (int(endX)-int(startX)) * mLocalCurrentMap.getResolution() / deltaY;
        double stepY = (int(endY)-int(startY)) * mLocalCurrentMap.getResolution() / deltaY;

        for(int i=1; i<= deltaY; i++)
        {
            double meterX = startX*mLocalCurrentMap.getResolution() + i*stepX;//为了统一格式。。虽然我知道这么写浪费计算资源
            double meterY = startY*mLocalCurrentMap.getResolution() + i*stepY;

            int cellX = meterX/mLocalCurrentMap.getResolution();
            int cellY = meterY/mLocalCurrentMap.getResolution();
            if(mLocalCurrentMap.getData(cellX, cellY) > mCostRayTrack)
                return false;
        }
        return true;
    }
}

/**
 * @brief RobotNavigator::publishDJGridMap
 * 调试使用，发布DJST地图
 */
void RobotNavigator::publishDJGridMap()
{
    nav_msgs::OccupancyGrid tempMap;
    tempMap.data.resize(mLocalCurrentMap.getSize());
    tempMap.info.height = mLocalCurrentMap.getHeight();
    tempMap.info.width = mLocalCurrentMap.getWidth();
    tempMap.info.resolution = mLocalCurrentMap.getResolution();
    tempMap.info.origin.position.x = mLocalCurrentMap.getOriginX();
    tempMap.info.origin.position.y = mLocalCurrentMap.getOriginY();
    tempMap.info.origin.position.z = mDebug_show_height;//故意提高 看的清楚

    double max_value=0.0;
    for(int i=0; i<mLocalCurrentMap.getSize(); i++)
    {
        if(mLocalCurrentPlan[i] > max_value)
            max_value = mLocalCurrentPlan[i];
        //ROS_INFO("value = %f", mLocalCurrentPlan[i]);
    }
    //ROS_INFO("CurrentPlan max_value = %f", max_value);

    for(int id=0; id<mLocalCurrentMap.getSize(); id++)
    {
        tempMap.data[id] = (mLocalCurrentPlan[id]/max_value)*255;
    }
    mLocalDJGridMapPublisher.publish(tempMap);



}

/**
 * @brief RobotNavigator::correctGoalPose
 * 假如goal点是被占用的，就需要调节goal的位置，直到找到一个free的点，
 * 工程上这样解决 但是，这个会有点问题。。这意味着目标点变了，只是变化不大
 * @return
 */
bool RobotNavigator::correctGoalPose()
{
    // Reset the plan
    int mapSize = mCurrentMap.getSize();
    for(int i = 0; i < mapSize; i++)
    {
        mCurrentPlan[i] = -1;
    }

    // Initialize the queue with the goal point
    Queue queue;
    Entry goal(0.0, mGoalPoint);
    queue.insert(goal);//这里都不检查可行性？？
    mCurrentPlan[mGoalPoint] = 0;

    Queue::iterator next;
    double linear = mCurrentMap.getResolution();

    // Do full search with Dijkstra-Algorithm
    while(!queue.empty())
    {
        // Get the nearest cell from the queue
        next = queue.begin();
        double distance = next->first;
        unsigned int index = next->second;
        queue.erase(next);

        if(mCurrentPlan[index] >= 0 && mCurrentPlan[index] < distance) continue;

        // Add all adjacent cells
        std::vector<unsigned int> neighbors = mCurrentMap.getNeighbors(index);
        for(unsigned int i = 0; i < neighbors.size(); i++)
        {
            if(mCurrentMap.isFree(neighbors[i]))
            {
                mGoalPoint = neighbors[i];
                return true;
            }else
            {
                double newDistance = distance + linear;
                if(mCurrentPlan[neighbors[i]] == -1)
                {
                    queue.insert(Entry(newDistance, neighbors[i]));
                    mCurrentPlan[neighbors[i]] = newDistance;
                }
            }
        }
    }
    return false;
}

/**
 * @brief RobotNavigator::stop
 * 停止
 */
void RobotNavigator::stop()
{
    csc_nav2d_operator::cmd stopMsg;
    stopMsg.Turn = 0;
    stopMsg.Velocity = 0;
    mCommandPublisher.publish(stopMsg);
    mStatus = NAV_ST_IDLE;
    mIsPaused = false;
    mIsStopped = false;
    mIsPaused = false;
}

/**
 * @brief RobotNavigator::generateLocalCommand
 * 运动输出
 * 注意，这里是使用射线跟随的方式进行局部方向优化。
 * @return
 */
bool RobotNavigator::generateLocalCommand()
{
    // Do nothing when paused
    if(mIsPaused || mIsNavPaused)
    {
        ROS_INFO_THROTTLE(1.0, "Navigator is paused and will not move now.");
        return true;
    }

    if(mStatus != NAV_ST_NAVIGATING )
    {
        ROS_WARN_THROTTLE(1.0, "Navigator has status %d when generateCommand() was called!", mStatus);
        return false;
    }

    // Generate direction command from plan
    unsigned int current_x = 0, current_y = 0;
    if(!mLocalCurrentMap.getCoordinates(current_x, current_y, mLocalStartPoint)) // || !mCurrentMap.isFree(mStartPoint))
    {
        ROS_ERROR("Plan execution failed, robot not in map!");
        return false;
    }

    unsigned int targetInSize = mLocalStartPoint;
    int steps = 1.25 / mLocalCurrentMap.getResolution();//看1.25m范围内的目标点，室内1.25比较好，狭窄环境能通过
    for(int i = 0; i < steps; i++)
    {
        unsigned int bestPoint = targetInSize;
        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(targetInSize);
        for(unsigned int i = 0; i < neighbors.size(); i++)
        {
            if(mLocalCurrentPlan[neighbors[i]] >= (unsigned int)0 && \
                    mLocalCurrentPlan[neighbors[i]] < mLocalCurrentPlan[bestPoint])
                bestPoint = neighbors[i];
        }
        targetInSize = bestPoint;
    }

    //raytrack优化局部目标

    //首先获得局部地图中的最优路径点
    unsigned int index = mLocalStartPoint;
    std::vector<unsigned int> pointsId;
    while(true)
    {
        unsigned int x = 0, y = 0;
        if(mLocalCurrentMap.getCoordinates(x,y,index))
        {
            pointsId.push_back(index);
        }
        if(mLocalCurrentPlan[index] == 0) break;//迭代到了goal

        unsigned int next_index = index;
        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(index);//获得周围8个领域
        for(unsigned int i = 0; i < neighbors.size(); i++)//查找8个领域里距离最小的值，也就是最大的梯度方向
        {
            if(mLocalCurrentPlan[neighbors[i]] >= 0 && mLocalCurrentPlan[neighbors[i]] < mLocalCurrentPlan[next_index])
                next_index = neighbors[i];
        }
        if(index == next_index) break;
        index = next_index;
    }

    unsigned int target = mLocalStartPoint;
    if(pointsId.size() > 1)//raytrack有效时
    {
        int i;
        //根据射线法，找到最远的目标点
        for(i=1; i<pointsId.size(); i++)
        {
            if(RayTrackisClean(pointsId[i]))
            {
                target = pointsId[i];
            }
            else
                break;
        }
        //之后判断 是否在1.25距离以内，1.25是室内运动的一个比较好的前瞻值，射线法应该找到比1.25更远的点
        if(mLocalCurrentPlan[target] > mLocalCurrentPlan[targetInSize])
        {
            target = targetInSize;//raytrack 追踪的节点没有0.75范围内找到的更加靠近goal
            ROS_INFO("raytrackGoal == localTargetGoal!");
        }
    }
    else
    {
        target = targetInSize;
    }
    //获得目标点距离车体中心的距离
    unsigned int targetX, targetY, startX, startY;
    mLocalCurrentMap.getCoordinates(startX, startY, mLocalStartPoint);
    mLocalCurrentMap.getCoordinates(targetX, targetY, target);

    double targetLenth = sqrt(pow(((int)startX-(int)targetX)*mLocalCurrentMap.getResolution(),2) + \
                              pow(((int)startY-(int)targetY)*mLocalCurrentMap.getResolution(),2));

    // Head towards (x,y)
    unsigned int x = 0, y = 0;
    if(!mLocalCurrentMap.getCoordinates(x, y, target))
    {
        ROS_ERROR("Plan execution failed, target pose not in map!");
        return false;
    }
    double map_angle = atan2((double)y - current_y, (double)x - current_x);

    double angle = map_angle - mCurrentDirection;
    if(angle < -PI) angle += 2*PI;
    if(angle > PI) angle -= 2*PI;

    // Create the command message
    csc_nav2d_operator::cmd msg;
    msg.Mode = 1;
    msg.Turn = (180.0/35) * angle / PI;  //角速度与偏差的角度成正比

    /*if(msg.Turn < -1) msg.Turn = -1;  //原地转向的
    if(msg.Turn >  1) msg.Turn = 1;*/

    //速度输出在0.3-0.6之间
    //角度和目标点位置决定当前速度输出，角度小速度大  目标点远速度大
    if(mLocalCurrentPlan[mLocalStartPoint] > mNavigationHomingDistance || mLocalCurrentPlan[mLocalStartPoint] < 0)
    {
        msg.Mode = 2;

        if(abs(msg.Turn) >= 1)
        {
            msg.Velocity = 0;
            //ROS_INFO("msg.Turn=%f !!",msg.Turn);
        }
        else
        {
            double speed_vel = (mSpeedMax - (mSpeedMax-mSpeedMin)*fabs(msg.Turn));//考虑转角
            //目标点在1.5m-3m之间系数线性变换。考虑距离
            double targetLenth_temp = targetLenth;
            if(targetLenth_temp > 3)
                targetLenth_temp = 3;
            if(targetLenth_temp < 1.5)
                targetLenth_temp = 1.5;
            double scale = 1;//(targetLenth_temp-1.5)*0.3333+0.5;

            speed_vel = speed_vel * scale;

            if(speed_vel < mSpeedMin)
                speed_vel = mSpeedMin;
            if(speed_vel > mSpeedMax)
                speed_vel = mSpeedMax;

            msg.Velocity = speed_vel;
        }
    }
    else//在距离目标3m左右开始变慢
    {
        msg.Mode = 1;
        msg.Velocity = mSpeedMin;
        if(abs(msg.Turn) >=1)
        {
            msg.Velocity = 0.0;
            //ROS_INFO("msg.Turn=%f !!",msg.Turn);
        }
    }

    msg.Turn = angle;  // 直接发送偏差角度
    mCommandPublisher.publish(msg);

    //调试使用 发布局部方向点的marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = mLocalCurrentMap.getOriginX() + (((double)x+0.5) * mLocalCurrentMap.getResolution());
    marker.pose.position.y = mLocalCurrentMap.getOriginY() + (((double)y+0.5) * mLocalCurrentMap.getResolution());
    marker.pose.position.z = mDebug_show_height;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = mCurrentMap.getResolution() * 1.0;
    marker.scale.y = mCurrentMap.getResolution() * 1.0;
    if(target == targetInSize)
        marker.scale.z = 3.0;
    else
        marker.scale.z = 6.0;//使用raytrack 用更长的mark表示
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    mLocalDirectionMarkerPublisher.publish(marker);

    return true;
}

/**
 * @brief RobotNavigator::receiveMoveGoal
 * 目标位置的服务程序，请求这个服务以后，在这个程序中会开始进行导航规划，直到达到目标点位置
 * @param goal
 */
void RobotNavigator::receiveMoveGoal(const csc_nav2d_navigator::MoveToPosition2DGoal::ConstPtr &goal)
{
    if(mStatus != NAV_ST_IDLE)
    {
        ROS_WARN("Navigator is busy!");
        mMoveActionServer->setAborted();
        return;
    }

    ROS_INFO("Received Goal: %.2f, %.2f (in frame '%s')", goal->target_pose.x, goal->target_pose.y, goal->header.frame_id.c_str());

    // Start navigating according to the generated plan
    Rate loopRate(FREQUENCY);
    unsigned int cycle = 0;
    bool reached = false;
    int recheckCycles = mMinReplanningPeriod * FREQUENCY;//这个变量暂时没用

    double targetDistance = (goal->target_distance > 0) ? goal->target_distance : mNavigationGoalDistance;//目标点的容忍距离
    //double targetAngle = (goal->target_angle > 0) ? goal->target_angle : mNavigationGoalAngle;//目标点的容忍角度
    double targetAngle = goal->target_pose.theta;

    // 这里是做全局规划
    if(cycle % recheckCycles == 0)
    {
        WallTime startTime = WallTime::now();
        mStatus = NAV_ST_NAVIGATING;

        // Create the plan for navigation
        mHasNewMap = false;
        if(!preparePlan())
        {
            ROS_ERROR("Prepare failed!");
            mMoveActionServer->setAborted();
            stop();
            return;
        }
        //换算到grid坐标系
        int mapX =  (double)(goal->target_pose.x - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
        int mapY =  (double)(goal->target_pose.y - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
        if(mapX < 0) mapX = 0;
        if(mapX >= (int)mCurrentMap.getWidth()) mapX = mCurrentMap.getWidth() - 1;
        if(mapY < 0) mapY = 0;
        if(mapY >= (int)mCurrentMap.getHeight()) mapY = mCurrentMap.getHeight() - 1;

        bool success = false;
        if(mCurrentMap.getIndex(mapX, mapY, mGoalPoint))//获得goal的id号，用于下一步规划
            success = createPlan();

        if(!success)
        {
            if(correctGoalPose())//如果没有规划成功，就矫正目标点的位置，重新规划，实际不会出现这个问题，人手点地图时，选择可以规划到的空白区域！
                success = createPlan();
        }

        if(!success)
        {
            ROS_ERROR("Planning failed!");
            mMoveActionServer->setAborted();
            stop();
            return;
        }

        WallTime endTime = WallTime::now();
        WallDuration d = endTime - startTime;
        ROS_INFO("Path planning took %.09f seconds, distance is %.2f m.", d.toSec(), mCurrentPlan[mStartPoint]);
    }

    while(true)
    {
        //构建局部地图，以机器人为中心1.5倍golbalCostMap的大小，中间叠加了带障碍物信息的局部costmap

        if(!setCurrentPosition())//获得在全局地图中的位置
        {
            ROS_ERROR("Navigation failed, could not get current position.");
            mMoveActionServer->setAborted();
            stop();
            return;
        }

        if(!createLocalMap())
        {
            ROS_ERROR("Prepare LocalMap failed!");
            mMoveActionServer->setAborted();
            stop();
            return;
        }
        if(!setLocalCurrentPosition())//获得在全局地图中的位置
        {
            ROS_ERROR("Navigation failed, could not get current position.");
            mMoveActionServer->setAborted();
            stop();
            return;
        }

        //基于全局路径找到局部地图里最边缘上的局部goal，
        //因为只叠加了1倍区域的costmap，外部0.5倍的环带区域是一定存在全局路径的。
        //为了防止局部地图中存在绕行的全局路线，在搜索时是从机器人当前位置搜索。
        if(!createLocalGoal())
        {
            ROS_ERROR("Prepare LocalGoal failed!");
            mMoveActionServer->setAborted();
            stop();
            return;
        }

        if(!createLocalPlan())
        {
            ROS_WARN("Prepare LocalPlan failed!");
            publishDJGridMap();
            mLocalPlanMissCounter = 5;
            mIsNavPaused = true;
            csc_nav2d_operator::cmd stopMsg;
            stopMsg.Mode = 0;//模式零是暂停，速度降为0,角度保持住
            stopMsg.Turn = 0;
            stopMsg.Velocity = 0;
            mCommandPublisher.publish(stopMsg);
        }
        else
        {
            if(mLocalPlanMissCounter>0)
            {
                mLocalPlanMissCounter--;
                ROS_INFO("mLocalPlanMissCounter=%d", mLocalPlanMissCounter);
            }
            else
            {
                mIsNavPaused = false;
            }
            publishDJGridMap();
            publishLocalPlan();//这一步必须执行。里面有保存当前帧的最短路径，用于优化下一次路径规划
        }

        // Check if we are asked to preempt
        if(!ok() || mMoveActionServer->isPreemptRequested() || mIsStopped)
        {
            ROS_INFO("Navigation has been preempted externally.");
            mMoveActionServer->setPreempted();//强制中断服务
            mIsStopped = false;
            stop();
            return;
        }

        // Where are we now
        mHasNewMap = false;
        if(!setCurrentPosition())//获得在全局地图中的位置
        {
            ROS_ERROR("Navigation failed, could not get current position.");
            mMoveActionServer->setAborted();
            stop();
            return;
        }



        // Are we already close enough?
        if(!reached && mCurrentPlan[mStartPoint] <= targetDistance && mCurrentPlan[mStartPoint] >= 0)
        {
            ROS_INFO("Reached target, now turning to desired direction.");
            reached = true;
        }

        if(reached)//reach以后开始调节角度，如果是阿卡慢模型，无法调节角度
        {
            double angle = targetAngle - mCurrentDirection;
            if(angle < -PI) angle += 2*PI;
            if(angle > PI) angle -= 2*PI;

            csc_nav2d_operator::cmd msg;

            if(fabs(angle) > mNavigationGoalAngle)
            {
                msg.Mode = 2;
                msg.Turn = (180.0/35) * angle / PI;  //角速度与偏差的角度成正比
                msg.Velocity = 0;
                mCommandPublisher.publish(msg);
            }
            else {
                msg.Mode = 2;
                msg.Turn = 0;
                msg.Velocity = 0;
                mCommandPublisher.publish(msg);

                ROS_INFO("auto run complete!");
                break;
            }
        }else
        {
            generateLocalCommand();
        }

        // Publish feedback via ActionServer
        if(cycle%10 == 0)
        {
            csc_nav2d_navigator::MoveToPosition2DFeedback fb;
            fb.distance = mCurrentPlan[mStartPoint];
            mMoveActionServer->publishFeedback(fb);
        }

        // Sleep remaining time
        cycle++;
        spinOnce();
        loopRate.sleep();
        if(cycle%10 == 0)
            ROS_INFO("time took %.4f s",loopRate.cycleTime().toSec());
        if(loopRate.cycleTime() > ros::Duration(1.0 / FREQUENCY))
            ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",FREQUENCY, loopRate.cycleTime().toSec());
    }

    // Set ActionServer suceeded
    ROS_INFO("Goal reached.");
    csc_nav2d_navigator::MoveToPosition2DResult r;
    r.final_pose.x = mCurrentPositionX;
    r.final_pose.y = mCurrentPositionY;
    r.final_pose.theta = mCurrentDirection;
    r.final_distance = mCurrentPlan[mStartPoint];
    mMoveActionServer->setSucceeded(r);
    stop();
}

/**
 * @brief RobotNavigator::isLocalized
 * 监听tf树，检查是否超时
 * @return
 */
bool RobotNavigator::isLocalized()
{
    return mTfListener.waitForTransform(mMapFrame, mRobotFrame, Time::now(), Duration(0.1));
}

/**
 * @brief RobotNavigator::setCurrentPosition
 * 根据tf，设置当前全局位置
 * @return
 */
bool RobotNavigator::setCurrentPosition()
{
    StampedTransform transform;
    try
    {
        mTfListener.lookupTransform(mMapFrame, mRobotFrame, Time(0), transform);
    }catch(TransformException ex)
    {
        ROS_ERROR("Could not get robot position: %s", ex.what());
        return false;
    }
    double world_x = transform.getOrigin().x();
    double world_y = transform.getOrigin().y();
    double world_theta = getYaw(transform.getRotation());

    unsigned int current_x = (world_x - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
    unsigned int current_y = (world_y - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
    unsigned int i;

    if(!mCurrentMap.getIndex(current_x, current_y, i))
    {
        if(mHasNewMap || !getMap() || !mCurrentMap.getIndex(current_x, current_y, i))
        {
            ROS_ERROR("Is the robot out of the map?");
            return false;
        }
    }
    mStartPoint = i;
    mCurrentDirection = world_theta;
    mCurrentPositionX = world_x;
    mCurrentPositionY = world_y;

    return true;
}

/**
 * @brief RobotNavigator::setLocalCurrentPosition
 * 根据全局位置设置局部坐标位置
 * @return
 */
bool RobotNavigator::setLocalCurrentPosition()
{
    unsigned int current_x;
    unsigned int current_y;
    unsigned int current_local_x;
    unsigned int current_local_y;
    unsigned int i;
    mCurrentMap.getCoordinates(current_x, current_y, mStartPoint);
    GridMapWorldToLocalCell(current_x, current_y, current_local_x, current_local_y);
    if(!mLocalCurrentMap.getIndex(current_local_x, current_local_y, i))
    {
        ROS_ERROR("Is the robot out of the LocalMap?wx=%d wy=%d lx=%d ly=%d",current_x, current_y,current_local_x, current_local_y);
        return false;
    }
    mLocalStartPoint = i;
    return true;
}
