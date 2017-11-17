/**
 *@brief 状态监测
 *@time 2017年10月10日
 */
#include <bpeer_status_monitor/bpeer_status_monitor.h>
#include <chrono>

int main ( int argc, char** argv )
{
	ros::init( argc, argv, "Q_status_monitor" );
	// delay run
//	chrono::steady_clock::time_point t100, t101;
//	chrono::duration< double > time_used;
//	t100 = chrono::steady_clock::now();
	sleep( 20 );
//	t101 = chrono::steady_clock::now();
//	time_used = chrono::duration_cast< chrono::duration<double > >( t101-t100 );
//	cout <<"sleep time： "<<time_used.count()<<endl;

	StatusMonitor statusMonitor;
	statusMonitor.status_process();

    cout << " todo here .." << endl;

	ros::spin();
	return 0;
}
