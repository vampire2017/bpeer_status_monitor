/**
 *@brief 状态监测
 *@time 2017年10月10日
 */
#include <bpeer_status_monitor/bpeer_status_monitor.h>


int main ( int argc, char** argv )
{
	ros::init( argc, argv, "Q_status_monitor" );

	StatusMonitor statusMonitor;
	statusMonitor.status_process();

    cout << " todo here .." << endl;

	ros::spin();
	return 0;
}
