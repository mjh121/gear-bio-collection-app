/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd All Rights Reserved
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "main_app.h"
#include <sensor.h>
#include <network/wifi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <time.h>
#include <pthread.h>
#include <storage.h>
#include <dirent.h>

#define BUF_LEN 64000

// **************************************************** Setting Value ****************************************************
char* gear_id = "gear_2"; // 디바이스 식별자 (gear_1, gear_2, gear_3)
char* dataType = "gear";
char* pc_Address = "192.168.15.202"; // IP Address
int pc_Port = 8100; // PORT Number (8100, 8101, 8102)
int hrm_first_min = 00; // 첫 번째 심박수 측정 시간 (분)
int hrm_second_min = 20; // 두 번째 심박수 측정 시간 (분)
int hrm_third_min = 40; // 세 번째 심박수 측정 시간 (분)
int hrm_time = 180; // 심박수 한번 측정 시간 (초 단위)
int hrm_Cycle = 1000; // 심박수 측정 주기 (ms 단위)
int gyro_Cycle = 100; // 자이로 측정 주기 (ms 단위)
int accel_Cycle = 100; // 가속도 측정 주기 (ms 단위)
int file_Cycle = 3600; // 파일 전송 주기 (초 단위)
                     // 데이터 기록 파일은 Gear에 /opt/usr/media/HRI_LOG 경로로 저장됨 (주기마다 파일 나눔, 기록된 순서부터 (unix time.txt -> unix time(+1).txt))
// ************************************************************************************************************************

Evas_Object *GLOBAL_DEBUG_BOX;
Evas_Object *start, *stop;
Evas_Object *conform;
static Evas_Object *event_label;
static Evas_Object *gyro_label;
static Evas_Object *accel_label;
sensor_listener_h hrm_listener;
sensor_listener_h gyro_listener;
sensor_listener_h accel_listener;
sensor_h hrm_sensor;
sensor_h gyro_sensor;
sensor_h accel_sensor;
static pthread_t p_thread;
static int thr_id;
static bool thr_exit = true;
static pthread_t p_thread_s;
static int thr_id_s;
static bool thr_exit_s = true;

struct sockaddr_in server_addr;

char h_d[10] = "0";
char g_x[10];
char g_y[10];
char g_z[10];
char a_x[10];
char a_y[10];
char a_z[10];
char temp[100];

time_t timer;
struct tm *t;

int current_year;
int current_mon;
int current_day;
int current_hour;
int current_min;
int current_sec;

int unixTime;
int check_time;

char file_name[100];
char file_check_name[100];
char file_send_name[100];

char file_path[100];
char file_path_start[30] = "/opt/usr/media/HRI_LOG/";
char file_path_end[30] = ".txt";

char gui_text_gyro[30];
char gui_text_acc[30];
char gui_text_hrm[30];

void _write_hri_data() // 데이터 전송
{
	int s;
	struct sockaddr_in server_addr;
	char buf[64000];
	FILE *fp;
	int status;
	int sentBytes = 0;

	if((s = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
//		exit(0);
		return;
	}

	/* 서버 소켓 구조체 설정 */
	bzero((char *)&server_addr, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(pc_Address);
	server_addr.sin_port = htons(pc_Port);

	/* 연결요청 */
	if(connect(s, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
		close(s);
//		dlog_print(DLOG_DEBUG, "USR_TAG-3", "[%s]", "FAUIL");
		return;
//		exit(0);
	}

	// file search&send START
	DIR *dir_info;
	struct dirent *dir_entry;

	dir_info = opendir("/opt/usr/media/HRI_LOG");
	if(dir_info != NULL) {
		while(dir_entry = readdir(dir_info)) {
			if(!strcmp(dir_entry->d_name, file_check_name)) {
//				dlog_print(DLOG_DEBUG, "USR_TAG-1", "[%s]", dir_entry->d_name);
				continue;
			} else {
//				dlog_print(DLOG_DEBUG, "USR_TAG-2", "[%s]", dir_entry->d_name);
				if(!strcmp(dir_entry->d_name, ".")) {
					continue;
				}
				if(!strcmp(dir_entry->d_name, "..")) {
					continue;
				}
				sprintf(file_send_name, "%s%s", file_path_start, dir_entry->d_name);
				fp = fopen(file_send_name, "rb");
				while(1) {
					status = fread(buf, 1, BUF_LEN, fp);
					if(status == 0) {
						strcpy(buf, "\r\n");
						send(s, buf, status, 0);
						break;
					}
					write(s, buf, status);
					sentBytes += status;
				}
				fclose(fp);
				remove(file_send_name);
				dlog_print(DLOG_DEBUG, "USR_TAG-s", "[%s]", file_send_name);
			}
		}
		closedir(dir_info);
	}

	close(s);
	// file search&send END
}

void write_file_reset(const char* filepath, const char* buf) // 파일 초기화
{
    FILE *fp;
    fp = fopen(filepath, "w");
    fputs(buf, fp);
    fclose(fp);
}

void write_file(const char* filepath, const char* buf) // 파일에 데이터 쓰기
{
    FILE *fp;
    fp = fopen(filepath, "a");
    fputs(buf, fp);
    fclose(fp);
}

void _sensor_stop_cb(void *data, Evas_Object *obj, void *event_info, sensor_listener_h listener)
{
    int error = sensor_listener_unset_event_cb(listener);
    error = sensor_listener_stop(listener);
    error = sensor_destroy_listener(listener);
}

void on_sensor_event(sensor_h sensor, sensor_event_s *event, void *user_data, sensor_listener_h listener) // 센서 이벤트 함수
{
	// Select a specific sensor with a sensor handle
	sensor_type_e type;
	sensor_get_type(sensor, &type);

	char temp_x[10];
	char temp_y[10];
	char temp_z[10];

	switch (type) {
	case SENSOR_HRM:
		sprintf(temp_x,"%0.0f", event->values[0]);
		// 심박 수 로그로 체크
//		dlog_print(DLOG_DEBUG, "USR_TAG", "%d, HRM event", current_min);
		//
		strcpy(h_d, temp_x);

//		// 심박수 GUI 출력
//		sprintf(gui_text_hrm,"%0.0f", event->values[0]);
//		elm_object_text_set(event_label, gui_text_hrm);
		break;
	case SENSOR_GYROSCOPE:
		sprintf(temp_x,"%0.2f", event->values[0]);
		strcpy(g_x, temp_x);
		sprintf(temp_y,"%0.2f", event->values[1]);
		strcpy(g_y, temp_y);
		sprintf(temp_z,"%0.2f", event->values[2]);
		strcpy(g_z, temp_z);

//		// 자이로 GUI 출력
//		sprintf(gui_text_gyro,"%0.2f %0.2f %0.2f", event->values[0], event->values[1], event->values[2]);
//		elm_object_text_set(gyro_label, gui_text_gyro);
		break;
	case SENSOR_ACCELEROMETER:
		sprintf(temp_x,"%0.2f", event->values[0]);
		strcpy(a_x, temp_x);
		sprintf(temp_y,"%0.2f", event->values[1]);
		strcpy(a_y, temp_y);
		sprintf(temp_z,"%0.2f", event->values[2]);
		strcpy(a_z, temp_z);

//		// 가속도 GUI 출력
//		sprintf(gui_text_acc, "%0.2f %0.2f %0.2f", event->values[0], event->values[1], event->values[2]);
//		elm_object_text_set(accel_label, gui_text_acc);

		// 파일에 쓰는 부분
		timer = time(NULL);
		t = localtime(&timer);

		current_year = t->tm_year + 1900;
		current_mon = t->tm_mon + 1;
		current_day = t->tm_mday;
		current_hour = t->tm_hour;
		current_min = t->tm_min;
		current_sec = t->tm_sec;

		unixTime = timer;

		sprintf(temp, "{\"deviceId\":\"%s\",\"dataType\":\"%s\",\"timestamp\":\"%d-%d-%dT%d:%d:%dZ\",\"regTime\":%d,\"gyro\":[%s,%s,%s],\"acc\":[%s,%s,%s],\"cardiac_value\":%s}\n", gear_id, dataType, current_year, current_mon, current_day, current_hour, current_min, current_sec, unixTime, g_x, g_y, g_z, a_x, a_y, a_z, h_d);
		strcpy(h_d, "0");
		write_file(file_name, temp);
		temp[0] = '\0';
		break;
	default:
		break;
	}
}

void _sensor_accuracy_changed_cb(sensor_h sensor, unsigned long long timestamp, sensor_data_accuracy_e accuracy, void *data) { }

void _sensor_start_cb(void *data, Evas_Object *obj, void *event_info, sensor_h sensor, sensor_type_e type, sensor_listener_h listener, unsigned int interval) // 센서가 지원되는지 확인
{
    void *user_data = NULL;

    // Retrieving a Sensor
    bool supported;
    int error = sensor_is_supported(type, &supported);
    if (error != SENSOR_ERROR_NONE) {
        return;
    }

    if(supported) { }

    // Get sensor list
    int count;
    sensor_h *list;

    error = sensor_get_sensor_list(type, &list, &count);
    if (error != SENSOR_ERROR_NONE) {
    } else {
        free(list);
    }

    error = sensor_get_default_sensor(type, &sensor);
    //센서 객체를 반환하는 API
    //SENSOR_PRESSURE를 전달하면, 파라미터에 Pressure 센서 객체가 반환
    if (error != SENSOR_ERROR_NONE) {
        return;
    }

    // Registering a Sensor Event
    error = sensor_create_listener(sensor, &listener);
    //이벤트 리스너를 생성하는 API
    //첫 번째 센서 객체를 전달하면 두번째 파라미터에 리스너 객체가 반환
    if (error != SENSOR_ERROR_NONE) {
        return;
    }

    int min_interval = 0;
    error = sensor_get_min_interval(sensor, &min_interval);
    if (error != SENSOR_ERROR_NONE) {
        return;
    }

    // Callback for sensor value change
    if(type == SENSOR_HRM) {
    	hrm_listener = listener;
    }
    error = sensor_listener_set_event_cb(listener, min_interval, on_sensor_event, listener);
    if (error != SENSOR_ERROR_NONE) {
        return;
    }

    // Registering the Accuracy Changed Callback
    error = sensor_listener_set_accuracy_cb(listener, _sensor_accuracy_changed_cb, user_data);
    if (error != SENSOR_ERROR_NONE) {
        return;
    }

    error = sensor_listener_set_interval(listener, interval);
    if (error != SENSOR_ERROR_NONE) {
        return;
    }

    error = sensor_listener_set_option(listener, SENSOR_OPTION_ALWAYS_ON);
    if (error != SENSOR_ERROR_NONE) {
        return;
    }

    error = sensor_listener_start(listener);
    if (error != SENSOR_ERROR_NONE) {
        return;
    }
}

static void win_delete_request_cb(void *data, Evas_Object *obj, void *event_info) // 앱 삭제 요청이 발생할 때 실행, 직접 호출 X
{
    elm_exit();
}

Eina_Bool _pop_cb(void *data, Elm_Object_Item *item)
{
    elm_win_lower(((appdata_s *)data)->win);
    return EINA_FALSE;
}

void* _hrm_timer(Evas_Object *box) // 심박수 측정 함수
{
	void *data = NULL;
	void *event_info = NULL;

	while(!thr_exit) {
		// 심박수 첫 번째 측정
		if(current_min == hrm_first_min) {
			_sensor_start_cb(data, box, event_info, hrm_sensor, SENSOR_HRM, hrm_listener, hrm_Cycle);
			sleep(hrm_time);
			_sensor_stop_cb(data, box, event_info, hrm_listener);
		}

		// 심박수 두 번째 측정
		if(current_min == hrm_second_min) {
			_sensor_start_cb(data, box, event_info, hrm_sensor, SENSOR_HRM, hrm_listener, hrm_Cycle);
			sleep(hrm_time);
			_sensor_stop_cb(data, box, event_info, hrm_listener);
		}

		// 심박수 세 번째 측정
		if(current_min == hrm_third_min) {
			_sensor_start_cb(data, box, event_info, hrm_sensor, SENSOR_HRM, hrm_listener, hrm_Cycle);
			sleep(hrm_time);
			_sensor_stop_cb(data, box, event_info, hrm_listener);
		}
	}

	pthread_exit((void *) 0);
}

void start_thread(Evas_Object *box)
{
	thr_exit = false;
	thr_id = pthread_create(&p_thread, NULL, _hrm_timer, box);
}

void end_thread()
{
	thr_exit = true;
	pthread_join(p_thread, (void**)NULL);
}

void file_name_change() // 다음 파일 지정해주는 함수
{
	check_time++;
	sprintf(file_path, "%s%d%s", file_path_start, check_time, file_path_end);
	write_file_reset(file_path, "");
	sprintf(file_name, "%s", file_path);
	sprintf(file_check_name, "%d%s", check_time, file_path_end);
}

void* socket_transfer() // 파일 변경 및 데이터 소켓 통신 함수
{
	while(!thr_exit_s) {
		sleep(file_Cycle);
		file_name_change();
//		_write_hri_data();
	}

	pthread_exit((void *) 0);
}

void start_thread_s()
{
	thr_exit_s = false;
	thr_id_s = pthread_create(&p_thread_s, NULL, socket_transfer, NULL);
}

void end_thread_s()
{
	thr_exit_s = true;
	pthread_join(p_thread_s, (void**)NULL);
}

static void create_base_gui(appdata_s *ad) // 화면을 구성하는 윈도우와 각종 컨테이너, 위젯을 생성하는 함수
{
	timer = time(NULL);
	t = localtime(&timer);

	current_year = t->tm_year + 1900;
	current_mon = t->tm_mon + 1;
	current_day = t->tm_mday;
	current_hour = t->tm_hour;
	current_min = t->tm_min;
	current_sec = t->tm_sec;

	check_time = timer;
	sprintf(file_path, "%s%d%s", file_path_start, check_time, file_path_end);
	write_file_reset(file_path, "");
	sprintf(file_name, "%s", file_path);

    // Setting the window
	// ad는 앱에서 아용하는 데이터를 저장하는 구조체
    ad->win = elm_win_util_standard_add(PACKAGE, PACKAGE);//윈도우 객체를 생성하는 API 입니다. 윈도우는 화면 레이아웃 최상위 오브젝트
    elm_win_conformant_set(ad->win, EINA_TRUE);//위젯의 캡션 텍스트를 변경하는 API, 레이블, 버튼, 엔트리에도 사용가능
    elm_win_autodel_set(ad->win, EINA_TRUE);//
    elm_win_indicator_mode_set(ad->win, ELM_WIN_INDICATOR_SHOW);
    elm_win_indicator_opacity_set(ad->win, ELM_WIN_INDICATOR_OPAQUE);
    evas_object_smart_callback_add(ad->win, "delete, request", win_delete_request_cb, NULL);//위젯 혹은 컨테이너 같은 스마트 오브젝트에 이벤트 콜백 함수를 지정하는 API.

    /* Create conformant */
    //Contormant는 화면에 새로운 영역이 추가되었을 때 윈도우 크기를 변경해주는 기능.
    //하나의 앱은 하나의 Conformant만 가져야한다.
    conform = elm_conformant_add(ad->win);

    evas_object_size_hint_weight_set(conform, EVAS_HINT_EXPAND, EVAS_HINT_EXPAND);//오브젝트의 크기를 대략적으로 지정하는 API.
    // 파라미터는 오브젝트, 수평 크기 힌트, 수직 크기 힌트 3가지로 구분, EVAS_HINT_EXPAND는 최대한 크게 지정한다는 의

    elm_win_resize_object_add(ad->win, conform);
    //window 오브젝트에 다른 오브젝트를 추가하면서 크기를 변경하는 API

    evas_object_show(conform);
    //오브젝트를 화면에 표시하는 API

    // Create main box
    Evas_Object *box = elm_box_add(conform); // Box 컨테이너를 생성하는 API
	elm_object_content_set(conform, box); // conform에 box layout 추가
	elm_box_horizontal_set(box, EINA_FALSE); // 배치방향 지정 API, EINA_TURE는 수평, EINA_FALSE는 수직
	evas_object_size_hint_align_set(box, EVAS_HINT_FILL, EVAS_HINT_FILL);
	evas_object_size_hint_weight_set(box, EVAS_HINT_EXPAND, EVAS_HINT_EXPAND);

    //Create Gyroscope_senser_data_value
	gyro_label = elm_label_add(box); // Label 위젯 생성 API
	evas_object_color_set(gyro_label, 000, 187, 000, 255);
	elm_object_text_set(gyro_label, "Measuring gyroscope");
	elm_box_pack_end(box,gyro_label); // Box 컨테이너에 새로운 객체 추가
	evas_object_show(gyro_label);

    //Create Accelerometer_senser_data_value
	accel_label = elm_label_add(box);
	evas_object_color_set(accel_label, 030, 144, 255, 255);
	elm_object_text_set(accel_label, "Measuring acceleration");
	elm_box_pack_end(box, accel_label);
	evas_object_show(accel_label);

    void *data = NULL;
    void *gyro_info = NULL;
    void *accel_info = NULL;

    start_thread(box); // 심박수 측정 스레드
	_sensor_start_cb(data, box, gyro_info, gyro_sensor, SENSOR_GYROSCOPE, gyro_listener, gyro_Cycle); // 자이로 이벤트 호출
	_sensor_start_cb(data, box, accel_info, accel_sensor, SENSOR_ACCELEROMETER, accel_listener, accel_Cycle); // 가속도 이벤트 호출
	start_thread_s(); // 파일 변경 및 데이터 소켓 통신 스레드

    //Create Hrm_senser_data_value
    event_label = elm_label_add(box);
    evas_object_color_set(event_label, 217, 000, 000, 255);
	elm_object_text_set(event_label, "Measuring heart rate");
	elm_box_pack_end(box, event_label);
	evas_object_show(event_label);

    // Show the window after base gui is set up
    evas_object_show(ad->win);
}

static bool app_create(void *data) // 앱 생성될때 불림
{
    create_base_gui((appdata_s *)data);

    return true;
}

int main(int argc, char *argv[]) // 메인 함수
{
    appdata_s ad;
    memset(&ad, 0x00, sizeof(appdata_s));

    ui_app_lifecycle_callback_s event_callback;
    memset(&event_callback, 0x00, sizeof(ui_app_lifecycle_callback_s));

    event_callback.create = app_create;

    int ret = ui_app_main(argc, argv, &event_callback, &ad);
    if (ret != APP_ERROR_NONE)

    return ret;
}
