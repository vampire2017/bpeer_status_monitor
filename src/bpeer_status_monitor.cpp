/**
 *@brief 状态监测
 *@time 2017年10月10日
 */
#include <bpeer_status_monitor/bpeer_status_monitor.h>

StatusMonitor::StatusMonitor()
{
	freq1 = 1;  // status 1hz
	freq2 = 30;  // report 30s
	bLaser_alive = false;
	mLaser_current_stamp.sec = 0;

	bOdom_alive = false;
	mOdom_current_stamp.sec = 0;

	mnStatus = 0;   //default 0 : 正常定位

	///init process
	//process: planner
	memset( planner_exe, 0, sizeof(planner_exe) );
	char exe_name_planner[] = "planner  ";  //进程名
	sprintf( planner_exe, "ps -ef | grep %s | grep -v grep | wc -l ", exe_name_planner );  //指令集

	//precess: amcl cov
	memset( amcl_exe, 0, sizeof(amcl_exe) );
	char exe_name_amcl[] = "amcl  ";  //进程名
	sprintf( amcl_exe, "ps -ef | grep %s | grep -v grep | wc -l ", exe_name_amcl );  //指令集

	//precess: 回冲
	memset( returnCharging_exe, 0, sizeof(returnCharging_exe) );
	char exe_name_returnCharging[] = "auto_charging ";  //进程名
	sprintf( returnCharging_exe, "ps -ef | grep %s | grep -v grep | wc -l ", exe_name_returnCharging );  //指令集

	//precess: 充电桩识别 @todo 充电桩识别的名字需要修改下
	memset( identifyCharging_exe, 0, sizeof(identifyCharging_exe) );
	char exe_name_identifyCharging[] = "identify_charging ";  //进程名
	sprintf( identifyCharging_exe, "ps -ef | grep %s | grep -v grep | wc -l ", exe_name_identifyCharging );  //指令集

	//precess: 自主建图
	memset( autoMapping_exe, 0, sizeof(autoMapping_exe) );
	char exe_name_autoMapping[] = "auto_mapping  ";  //进程名
	sprintf( autoMapping_exe, "ps -ef | grep %s | grep -v grep | wc -l ", exe_name_autoMapping );  //指令集

	//precess: tf2topic
	memset( tf2topic_exe, 0, sizeof(tf2topic_exe) );
	char exe_name_tf2topic[] = "pubTf2Topic ";  //进程名
	sprintf( tf2topic_exe, "ps -ef | grep %s | grep -v grep | wc -l ", exe_name_tf2topic );  //指令集

}

void StatusMonitor::status_process()
{
	ros::NodeHandle nh_private("~");

	status_Q_pub_ = nh_.advertise< bprobot::msg_Q_status_monitor >( "/Q_monitor_status", 1 );

	laser_sub_ = nh_.subscribe< sensor_msgs::LaserScan >( "/scan", 2,
	                            [this](const sensor_msgs::LaserScanConstPtr& laser_data){
//		                            cout << "laser in.. " << endl;
		                            bLaser_alive = laser_data->header.seq;
		                            mLaser_current_stamp = laser_data->header.stamp;
//		                            cout << "live time: " << mLaser_current_stamp << " s" << endl;
//		                            cout << "alive: " << bLaser_alive << endl;  //test
	                            } );

	odom_sub_ = nh_.subscribe< nav_msgs::Odometry >( "/odom", 2,
							[this](const nav_msgs::OdometryConstPtr& odom_data){
//								cout << "odom in.. " << endl;
								mOdom_current_stamp = odom_data->header.stamp;
								bOdom_alive = odom_data->header.seq;
//								cout << "live time: " << mOdom_current_stamp << " s" << endl;
//								cout << "alive: " << bOdom_alive << endl;   //test
							} );
	/**
	 * @brief:   0->正常定位中(扫地等); 1->地图构建中; 2->重定位中; 3->回充中; 4->模块异常.
	 * @brief1: -1->建图失败;  0->开始建图;  1->建图中;  2->建图完成.
	 * @brief2: -1->回充失败;  0->回充开始;  1->回充中;  2->回充完成.
	 * @brief3: -2->重定位失败; -1->定位失败; 0->定位正常; 1->重定位开始; 2->重定位中; 3->重定位完成.
	 * @param mnStatus
	 */
	autoCreateMap_sub_ = nh_.subscribe< std_msgs::Int8 >( "/Q_auto_create_map", 2,
		[this](const std_msgs::Int8ConstPtr& msg){
			switch ( msg->data )
			{
				case -1:
					mnStatus = 4;
					break;
				case 0:
					mnStatus = 1;
					break;
				case 1:
					mnStatus = 1;
					break;
				case 2:
					mnStatus = 0;
				default: break;
			}
		});

	autoCharge_sub_ = nh_.subscribe< std_msgs::Int8 >( "/Q_auto_charge", 2,
		[this](const std_msgs::Int8ConstPtr& msg){
			switch ( msg->data )
			{
				case -1:
					mnStatus = 4;
					break;
				case 0:
					mnStatus = 3;
					break;
				case 1:
					mnStatus = 3;
					break;
				case 2:
					mnStatus = 0;
					break;
				default: break;
			}
		});

	localization_sub_ = nh_.subscribe< std_msgs::Int8 >( "/Q_localization", 2,
		[this](const std_msgs::Int8ConstPtr& msg){
			switch ( msg->data )
			{
				case -2:
					mnStatus = 4;
					break;
				case -1:
					mnStatus = 4;
					break;
				case 0:
					mnStatus = 0;
					break;
				case 1:
					mnStatus = 2;
					break;
				case 2:
					mnStatus = 2;
					break;
				case 3:
					mnStatus = 0;
					break;
				default: break;
			}
		});

	statusReport_pub_ =nh_.advertise< bprobot::msg_A_STATUS_REPORT >( "/A_STATUS_REPORT", 1 );

	timer1_ = nh_private.createTimer( ros::Duration(1.0/freq1), &StatusMonitor::process_receiveDataSpin, this );
	timer2_ = nh_private.createTimer( ros::Duration(freq2), &StatusMonitor::process_reportStatusSpin, this );

}

void StatusMonitor::process_reportStatusSpin(const ros::TimerEvent &e)
{
	ROS_INFO("living time...");

	bprobot::msg_A_STATUS_REPORT statusReport;
	statusReport.time = ros::Time::now();
	statusReport.module = "Q";
	statusReport.type = 4;

	// 模块异常report
	// TODO planner amcl IdentifyCharging_status tf2topic --> always alive;
	if ( !mbPlanner_status || !mbAmcl_status || !mbReturnCharging_status || !mbIdentifyCharging_status )
		mnStatus = 4;
	statusReport.data = "{\"stat\" : "
	                    + std::to_string( mnStatus )
	                    + "}";
	statusReport_pub_.publish( statusReport );
}

void StatusMonitor::process_receiveDataSpin(const ros::TimerEvent &e)
{
	// pub -> status
	bprobot::msg_Q_status_monitor statusMonitorPub;
	statusMonitorPub.stamp = ros::Time::now();
	statusMonitorPub.laser_data = bLaser_alive;  //true: 有数据  false:无数据
	statusMonitorPub.odom_data = bOdom_alive;

//	cout << "laser time: " << statusMonitorPub.stamp.toSec() << endl;
//	cout << "mLaser_current_stamp: " << mLaser_current_stamp.toSec() << endl;

	///sensor
	if( !bLaser_alive )
		if ( statusMonitorPub.stamp.toSec() > mLaser_current_stamp.toSec() + 2 )  // >2 s
			statusMonitorPub.laser_alive = 0;
		else
			statusMonitorPub.laser_alive = 1;
	else
		statusMonitorPub.laser_alive = 1;

	if( !bOdom_alive )
		if ( statusMonitorPub.stamp.toSec() > mOdom_current_stamp.toSec() + 2 )  // >2 s
			statusMonitorPub.odom_alive = 0;
		else
			statusMonitorPub.odom_alive = 1;
	else
		statusMonitorPub.odom_alive = 1;

	///process
	monitor_process_alive();

	statusMonitorPub.process_planner = mbPlanner_status;
	statusMonitorPub.process_amcl = mbAmcl_status;
	statusMonitorPub.process_returnCharging = mbReturnCharging_status;
	statusMonitorPub.process_identifyCharging = mbIdentifyCharging_status;
	statusMonitorPub.process_autoMapping = mbAutoMapping_status;
	statusMonitorPub.process_pubTf2Topic = mbTf2Topic_status;
	status_Q_pub_.publish( statusMonitorPub );

	// clear this status
	bLaser_alive = false;
	bOdom_alive = false;
}

void StatusMonitor::monitor_process_alive()
{
	mbPlanner_status = process_is_ok( planner_exe );
	mbAmcl_status = process_is_ok( amcl_exe );
	mbReturnCharging_status = process_is_ok( returnCharging_exe );
	mbIdentifyCharging_status = process_is_ok( identifyCharging_exe );
	mbAutoMapping_status = process_is_ok( autoMapping_exe );
	mbTf2Topic_status = process_is_ok( tf2topic_exe );
}

bool StatusMonitor::process_is_ok(const char *node)
{
	bool status_ret = false;

//	cout << " process: " << node << endl;

	FILE *mProcess = popen( node, "r" );
	if ( mProcess == NULL )
		return false;

	char buff[512];
	memset( buff, 0, sizeof(buff) );
	fgets( buff, 512, mProcess );
	int flag = atoi( buff );

//	cout << "test:..  " << flag<< endl;

	if( !flag  )
	{
//		cout << " --failed-- " << endl;
		status_ret = false;
	}
	else
	{
//		cout << " good **** " <<  endl;
		status_ret = true;
	}
	pclose( mProcess );

	return status_ret;
}



