#include "webServerClient.h"

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "cJSON.h"

#include "../websocket/include/websocket_server.h"

#include "../controllers/controllers.h"

extern volatile int robotEnable; 
extern volatile PStructure Position;
extern volatile PDStructure Angle;
volatile double setPosition;
volatile double setTestValue;
volatile double setVelocity;
volatile double setVelocityTurnFactor;

extern MessageBufferHandle_t xMessageBufferToClient;
extern volatile RobotMode currentMode;

// Parse messages
void parse_message(const char* msg) {
    const static char* TAG = "JSON";

    cJSON *json = cJSON_Parse(msg);
    if (json == NULL) {
        ESP_LOGE("JSON", "JSON message parsing error");
        return;
    }

    // Get ID Mode
    cJSON *id = cJSON_GetObjectItem(json, "id");
    if (!cJSON_IsString(id)) {
        ESP_LOGE(TAG, "Invalid message ID");
        cJSON_Delete(json);
        return;
    }

    // Checking the robot's power-on status
    cJSON *enable = cJSON_GetObjectItem(json, "robotEnable");
    if (cJSON_IsNumber(enable)) {
        robotEnable = (int)enable->valuedouble;
    }

    // Modes Handling
	if (strcmp(id->valuestring, "balanceMode") == 0) 
	{
		currentMode = BALANCE_MODE;
    } 
	else if (strcmp(id->valuestring, "positionMode") == 0) 
	{
        cJSON *webPosition = cJSON_GetObjectItem(json, "position");
        if (cJSON_IsNumber(webPosition)) {
			currentMode = POSITION_MODE;
			//Position.MeasuredValue = 0;
            //setPosition = webPosition->valuedouble;
			Position.SetpointValue = -webPosition->valuedouble; 
        }
    } 
	else if (strcmp(id->valuestring, "sensorMode") == 0) 
	{
        cJSON *webSensorPosition = cJSON_GetObjectItem(json, "sensorPosition");
        if (cJSON_IsNumber(webSensorPosition)) {
			currentMode = SENSOR_MODE;
            setPosition = webSensorPosition->valuedouble;
        }
    }
	else if (strcmp(id->valuestring, "remoteMode") == 0) 
	{
        cJSON *webVelocity = cJSON_GetObjectItem(json, "velocity");
		cJSON *webVelocityTurnFactor = cJSON_GetObjectItem(json, "turnFactor");
        if (cJSON_IsNumber(webVelocity) && cJSON_IsNumber(webVelocityTurnFactor)) 
		{
			currentMode = REMOTE_MODE;
			setVelocity = webVelocity->valuedouble;
			setVelocityTurnFactor = webVelocityTurnFactor->valuedouble;
        }
    } 
	else if (strcmp(id->valuestring, "quadrantSequenceMode") == 0) 
	{
		setVelocity = -0.4;
		currentMode = QUADRANT_SEQUENCE_MODE;
    } 
	else if (strcmp(id->valuestring, "stop") == 0) 
	{
        robotEnable = 0;
    }
	else if (strcmp(id->valuestring, "ctrlMode") == 0)
	{
        cJSON *webKpAngle = cJSON_GetObjectItem(json, "KpAngle");
		cJSON *webKdAngle = cJSON_GetObjectItem(json, "KdAngle");
		if (cJSON_IsNumber(webKpAngle) && cJSON_IsNumber(webKdAngle)) 
		{
			currentMode = CTRL_MODE;
			Angle.Kp = webKpAngle->valuedouble;
			Angle.Kd = webKdAngle->valuedouble;
        }
	}
	else if (strcmp(id->valuestring, "testMode") == 0) 
	{
        cJSON *webAngle = cJSON_GetObjectItem(json, "tmp");
        if (cJSON_IsNumber(webAngle)) {
			//currentMode = TEST_MODE_POSITION;
			currentMode = TEST_MODE_VELOCITY;
            setTestValue = webAngle->valuedouble;
        }
    } 
	else {
        ESP_LOGE(TAG, "Invalid mode ID");
    }

    // Memory cleaning
    cJSON_Delete(json);
}

void client_task(void* pvParameters) {
	const static char* TAG = "clientTask";
	int32_t task_parameter = (int32_t)pvParameters;
	ESP_LOGI(TAG, "Starting. task_parameter=0x%"PRIx32, task_parameter);

	char cRxBuffer[512];
	char DEL = 0x04;
	char outBuffer[100];

	while (1) {
		size_t readBytes = xMessageBufferReceive(xMessageBufferToClient, cRxBuffer, sizeof(cRxBuffer), portMAX_DELAY );
		ESP_LOGD(TAG, "readBytes=%d", readBytes);
		ESP_LOGD(TAG, "cRxBuffer=[%.*s]", readBytes, cRxBuffer);
		cJSON *root = cJSON_Parse(cRxBuffer);
		if (cJSON_GetObjectItem(root, "id")) {
			char *id = cJSON_GetObjectItem(root,"id")->valuestring;
			ESP_LOGD(TAG, "id=[%s]",id);

			if ( strcmp (id, "data-request") == 0) {
				double p_ref = cJSON_GetObjectItem(root,"p_ref")->valuedouble;
				double p = cJSON_GetObjectItem(root,"p")->valuedouble;
				double v_ref = cJSON_GetObjectItem(root,"v_ref")->valuedouble;
				double v = cJSON_GetObjectItem(root,"v")->valuedouble;
				double O_ref = cJSON_GetObjectItem(root,"O_ref")->valuedouble;
				double O = cJSON_GetObjectItem(root,"O")->valuedouble;
				double a = cJSON_GetObjectItem(root,"a")->valuedouble;
				//ESP_LOGD(TAG,"roll=%f", roll);

				sprintf(outBuffer,"DATA%c%f%c%f%c%f%c%f%c%f%c%f%c%f", DEL, p_ref, DEL, p, DEL, v_ref, DEL, v, DEL, O_ref, DEL, O, DEL, a);
				
				ESP_LOGD(TAG, "outBuffer=[%s]", outBuffer);
				ws_server_send_text_all(outBuffer,strlen(outBuffer));

				ESP_LOGD(TAG,"free_size:%d %d", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_free_size(MALLOC_CAP_32BIT));

			} // end if
		} // end if

		// Delete a cJSON structure
		cJSON_Delete(root);

	} // end while

	// Never reach here
	vTaskDelete(NULL);
}

static QueueHandle_t client_queue;
extern MessageBufferHandle_t xMessageBufferToClient;

const static int client_queue_size = 10;

// handles websocket events
void websocket_callback(uint8_t num,WEBSOCKET_TYPE_t type,char* msg,uint64_t len) {
	const static char* TAG = "websocket_callback";

	switch(type) {
		case WEBSOCKET_CONNECT:
			ESP_LOGI(TAG,"client %i connected!",num);
			break;
		case WEBSOCKET_DISCONNECT_EXTERNAL:
			ESP_LOGI(TAG,"client %i sent a disconnect message",num);
			break;
		case WEBSOCKET_DISCONNECT_INTERNAL:
			ESP_LOGI(TAG,"client %i was disconnected",num);
			break;
		case WEBSOCKET_DISCONNECT_ERROR:
			ESP_LOGI(TAG,"client %i was disconnected due to an error",num);
			break;
		case WEBSOCKET_TEXT:
			if(len) { // if the message length was greater than zero
				//ESP_LOGI(TAG, "got message length %i: %s", (int)len, msg);
				parse_message(msg);
				size_t xBytesSent = xMessageBufferSendFromISR(xMessageBufferToClient, msg, len, NULL);
				if (xBytesSent != len) {
					ESP_LOGE(TAG, "xMessageBufferSend fail");
				}
			}
			break;
		case WEBSOCKET_BIN:
			ESP_LOGI(TAG,"client %i sent binary message of size %"PRIu32":\n%s",num,(uint32_t)len,msg);
			break;
		case WEBSOCKET_PING:
			ESP_LOGI(TAG,"client %i pinged us with message of size %"PRIu32":\n%s",num,(uint32_t)len,msg);
			break;
		case WEBSOCKET_PONG:
			ESP_LOGI(TAG,"client %i responded to the ping",num);
			break;
	}
}

// serves any clients
static void http_server(struct netconn *conn) {
	const static char* TAG = "http_server";
	const static char HTML_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";
	const static char ERROR_HEADER[] = "HTTP/1.1 404 Not Found\nContent-type: text/html\n\n";
	const static char JS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/javascript\n\n";
	const static char CSS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/css\n\n";
	//const static char PNG_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/png\n\n";
	const static char ICO_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/x-icon\n\n";
	//const static char PDF_HEADER[] = "HTTP/1.1 200 OK\nContent-type: application/pdf\n\n";
	//const static char EVENT_HEADER[] = "HTTP/1.1 200 OK\nContent-Type: text/event-stream\nCache-Control: no-cache\nretry: 3000\n\n";
	struct netbuf* inbuf;
	static char* buf;
	static uint16_t buflen;
	static err_t err;

	// default page
	extern const uint8_t root_html_start[] asm("_binary_root_html_start");
	extern const uint8_t root_html_end[] asm("_binary_root_html_end");
	const uint32_t root_html_len = root_html_end - root_html_start;

	// main.js
	extern const uint8_t main_js_start[] asm("_binary_main_js_start");
	extern const uint8_t main_js_end[] asm("_binary_main_js_end");
	const uint32_t main_js_len = main_js_end - main_js_start;

	// main.css
	extern const uint8_t main_css_start[] asm("_binary_main_css_start");
	extern const uint8_t main_css_end[] asm("_binary_main_css_end");
	const uint32_t main_css_len = main_css_end - main_css_start;

	// favicon.ico
	extern const uint8_t favicon_ico_start[] asm("_binary_favicon_ico_start");
	extern const uint8_t favicon_ico_end[] asm("_binary_favicon_ico_end");
	const uint32_t favicon_ico_len = favicon_ico_end - favicon_ico_start;

	// error page
	extern const uint8_t error_html_start[] asm("_binary_error_html_start");
	extern const uint8_t error_html_end[] asm("_binary_error_html_end");
	const uint32_t error_html_len = error_html_end - error_html_start;

	netconn_set_recvtimeout(conn,1000); // allow a connection timeout of 1 second
	ESP_LOGI(TAG,"reading from client...");
	err = netconn_recv(conn, &inbuf);
	ESP_LOGI(TAG,"read from client");
	if(err==ERR_OK) {
		netbuf_data(inbuf, (void**)&buf, &buflen);
		if(buf) {

			ESP_LOGD(TAG, "buf=[%s]", buf);
			// default page
			if		 (strstr(buf,"GET / ")
					&& !strstr(buf,"Upgrade: websocket")) {
				ESP_LOGI(TAG,"Sending /");
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER)-1,NETCONN_NOCOPY);
				netconn_write(conn, root_html_start,root_html_len,NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}

			// default page websocket
			else if(strstr(buf,"GET / ")
					 && strstr(buf,"Upgrade: websocket")) {
				ESP_LOGI(TAG,"Requesting websocket on /");
				ws_server_add_client(conn,buf,buflen,"/",websocket_callback);
				netbuf_delete(inbuf);
			}

			else if(strstr(buf,"GET /main.js ")) {
				ESP_LOGI(TAG,"Sending /main.js");
				netconn_write(conn, JS_HEADER, sizeof(JS_HEADER)-1,NETCONN_NOCOPY);
				netconn_write(conn, main_js_start, main_js_len,NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}

			else if(strstr(buf,"GET /main.css ")) {
				ESP_LOGI(TAG,"Sending /main.css");
				netconn_write(conn, CSS_HEADER, sizeof(CSS_HEADER)-1,NETCONN_NOCOPY);
				netconn_write(conn, main_css_start, main_css_len,NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}

			else if(strstr(buf,"GET /favicon.ico ")) {
				ESP_LOGI(TAG,"Sending favicon.ico");
				netconn_write(conn,ICO_HEADER,sizeof(ICO_HEADER)-1,NETCONN_NOCOPY);
				netconn_write(conn,favicon_ico_start,favicon_ico_len,NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}

			else if(strstr(buf,"GET /")) {
				ESP_LOGE(TAG,"Unknown request, sending error page: %s",buf);
				netconn_write(conn, ERROR_HEADER, sizeof(ERROR_HEADER)-1,NETCONN_NOCOPY);
				netconn_write(conn, error_html_start, error_html_len,NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}

			else {
				ESP_LOGE(TAG,"Unknown request");
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
		}
		else {
			ESP_LOGI(TAG,"Unknown request (empty?...)");
			netconn_close(conn);
			netconn_delete(conn);
			netbuf_delete(inbuf);
		}
	}
	else { // if err==ERR_OK
		ESP_LOGI(TAG,"error on read, closing connection");
		netconn_close(conn);
		netconn_delete(conn);
		netbuf_delete(inbuf);
	}
}

// receives clients from queue, handles them
void server_handle_task(void* pvParameters) {
	const static char* TAG = "server_handle_task";
	struct netconn* conn;
	ESP_LOGI(TAG,"Starting");
	for(;;) {
		xQueueReceive(client_queue,&conn,portMAX_DELAY);
		if(!conn) continue;
		http_server(conn);
	}
	vTaskDelete(NULL);
}

// handles clients when they first connect. passes to a queue
void server_task(void* pvParameters) {
	const static char* TAG = "server_task";
	char url[64];
	sprintf(url, "http://%s", "192.168.1.1");
	ESP_LOGI(TAG, "Starting server on %s", url);

	struct netconn *conn, *newconn;
	static err_t err;
	client_queue = xQueueCreate(client_queue_size,sizeof(struct netconn*));
	configASSERT( client_queue );

	
	UBaseType_t PriorityGet = uxTaskPriorityGet(NULL);
	ESP_LOGI(TAG, "PriorityGet=%d", PriorityGet);
	xTaskCreate(&server_handle_task, "server_handle", 1024*3, NULL, PriorityGet, NULL);


	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn,NULL,80);
	netconn_listen(conn);
	ESP_LOGI(TAG,"server listening");
	do {
		err = netconn_accept(conn, &newconn);
		ESP_LOGI(TAG,"new client");
		if(err == ERR_OK) {
			xQueueSendToBack(client_queue,&newconn,portMAX_DELAY);
			//http_server(newconn);
		}
	} while(err == ERR_OK);
	netconn_close(conn);
	netconn_delete(conn);
	ESP_LOGE(TAG,"task ending, rebooting board");
	esp_restart();
}