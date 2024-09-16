#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ini-parse.h"

#define MAX_LINE_LEN 1024
#define CMD_LEN 16
char *ini_info_str[]={
"UNDEFINE",
"RJ45",
"RS485",
"DATA UNDEFINE",
"FULL DATA",
"SIMPLE DATA"
};

static char *frame_title[]={"FRAME1","FRAME2","FRAME3","FRAME4","FRAME5","FRAME6","FRAME7","FRAME8","FRAME9","FRAME10"};
static unsigned char *do1_on=NULL;
static unsigned char *do2_on=NULL;
static unsigned char *do1_off=NULL;
static unsigned char *do2_off=NULL;
static unsigned char rs485_cmd_len=0;
static int ir8062_connectivity=CONNECTION_UNDEFINE;
static tFrame_t tFrame[FRAME_NUMBER]={0};
system_param_t ini_params;



static int str2hex(char *str, unsigned char **cmd) {
	char *token;
	const char delimiter[] = ",";
	int cmd_len,i ;
	unsigned int dio_cmd[CMD_LEN]={0};
	unsigned int hex_value;
	printf("DIO comomand str = %s\n", str);

	// 使用strtok函数分割字符串
	cmd_len= 0;
	token = strtok(str, delimiter);

	while (token != NULL) {
		cmd_len++;
		if (token[0]==0x20) 
			token=token+1;
#if 0
	char *endptr;
	hex_value = strtol(token, &endptr, 16);
	if (*endptr != '\0') {
		printf("Invalid input: %s\n", token);
		return 1;
	}
	dio_cmd[cmd_len-1] = (unsigned int)hex_value;
	printf("%x ", dio_cmd[cmd_len-1]);

#else
		sscanf(token, "0x%x", &hex_value); // &dio_cmd[cmd_len-1]);
        	dio_cmd[cmd_len-1] = (unsigned int)hex_value;
		printf("%x ", dio_cmd[cmd_len-1]);
#endif
		token = strtok(NULL, delimiter);
	}

	printf("\nDO Command(%d): ",cmd_len);
	rs485_cmd_len=cmd_len;
	if (cmd_len) {
		*cmd = (unsigned char *)malloc((cmd_len)*sizeof(unsigned char));
		if (*cmd == NULL) {
			printf("ERROR : alloc do command failed\n");
			return 0;
		}
		for (i=0;i<cmd_len;i++) {
			(*cmd)[i]=dio_cmd[i];
			printf("0x%x , ", (*cmd)[i]);
		}
		printf("\n");
	}
	else 
		printf("ERROR : NO DIO Command found\n");
	
	return cmd_len;
}

void print_dio_cmd(unsigned char *data) {
	int i;
	int len = get_rs485_cmd_len();
	printf("=== DO CMD === len = %d : ", len);
	if (data == NULL) {
		printf("cmd data is null\n");
		return ;
	}
	for (i=0;i<len;i++) {
		printf("0x%x,",data[i]);
	}
	printf("\n");
}
static void set_conn_type(char *key, char *value){
	int val=0;
	if (strstr(key,"mode")!=NULL){
		if (strstr(value,"RJ45")!=NULL)
			ini_params.conn_type=RJ45;
		else if (strstr(value,"RS485")!=NULL)
			ini_params.conn_type=RS485;
		else {
			ini_params.conn_type=RJ45;
			printf("Error : Unknow connection mode %s. Use RJ45 as default\n", value);
		}
		printf("mode = %s\n", value);
	}
	else if (strstr(key,"data")!=NULL) {
		if (strstr(value,"SIMPLE")!=NULL) {
			ini_params.post_type = POST_SIMPLE_DATA;
		}
		else if (strstr(value, "FULL")) {
			ini_params.post_type = POST_FULL_DATA;
		}
		printf("data = %s\n", value);
	}
	else if (strstr(key,"alarm")!=NULL) {
		if (strstr(value,"on")!=NULL) {
			val = 1;
		}
		else if (strstr(value, "off")) {
			val = 0;
		}
		printf("alarm = %s\n", value);
		ini_params.alarm=val;
	}
	else if ((strstr(key,"over_temperature")!=NULL) && ini_params.alarm) {
		if (value==NULL) {
			printf("Over Temperature is NULL\n");
			val=0xffff;
		}
		else { 
			val = atoi(value);
		}
		ini_params.over_alarm1=val;
		printf("over temperature = %d\n", val);
	}
	else if (strstr(key,"under_temperature")!=NULL) {
		if (value==NULL) {
			printf("Under Temperature is NULL\n");
			val=-1;
		}
		else {
			val = atoi(value);
		}
		ini_params.under_alarm=val;
		printf("under temperature = %d\n", ini_params.under_alarm);

		//eth_config.under_temperature=val;
	}
	else if (strstr(key,"alert_temperature")!=NULL) {
		if (value==NULL) {
			printf("Alert Temperature is NULL\n");
			val=-1;
		}
		else {
			val = atoi(value);
		}
		ini_params.over_alarm2=val;
		printf("Alert temperature = %d\n", val);

		//eth_config.under_temperature=val;
	}
	else if (strstr(key,"do1on")!=NULL) {
		if (value==NULL) {
			printf("DO 1 CMD is NULL\n");
			do1_on=NULL;
		}
		else {
			val=str2hex(value, &do1_on);
		}
	}
	else if (strstr(key,"do1off")!=NULL) {
		if (value==NULL) {
			printf("DO 1 OFF CMD is NULL\n");
			do1_off=NULL;
		}
		else {
			val=str2hex(value, &do1_off);
		}
	}
	else if (strstr(key,"do2on")!=NULL) {
		if (value==NULL) {
			printf("DO 2 CMD is NULL\n");
			do2_on=NULL;
		}
		else {
			val=str2hex(value, &do2_on);
		}
	}
	else if (strstr(key,"do2off")!=NULL) {
		if (value==NULL) {
			printf("DO 2 Off CMD is NULL\n");
			do2_off=NULL;
		}
		else {
			val=str2hex(value, &do2_off);
		}
	}
}
static void set_frame_params(int id, char *key, char *value)
{
	int val=0;
	if (tFrame[id].status==FRAME_PARAMS_ERROR)
	{
		printf("ERROR: Wrong Frame%d params, skip...(key=%s, vlaue=%s)\n", id+1, key,value);
		return;
	}
	tFrame[id].status=FRAME_ENABLE;
	if (strstr(key,"x")!=NULL) {
		val=atoi(value);
		if (val<XMAX)
			tFrame[id].x=val;
		else {
			printf("ERROR: frame%d X=%d value over %d\n",id+1,val,XMAX);
			tFrame[id].status=FRAME_PARAMS_ERROR;
		}
	}
	else if (strstr(key,"y")!=NULL) {
		val=atoi(value);
		if (val<YMAX)
			tFrame[id].y=val;
		else {
			printf("ERROR: frame%d Y=%d axis value over %d\n",id+1,val,YMAX);
			tFrame[id].status=FRAME_PARAMS_ERROR;
		}

	}
	else if (strstr(key,"w")!=NULL) {
		val=atoi(value);
		if ((val+tFrame[id].x)<XMAX)
			tFrame[id].w=val;
		else {
			printf("ERROR: frame%d X end =%d, over %d\n",id+1,val+tFrame[id].x,XMAX);
			tFrame[id].status=FRAME_PARAMS_ERROR;
		}

	}
	else if (strstr(key,"h")!=NULL) {
		val=atoi(value);
		if ((val+tFrame[id].y)<YMAX)
			tFrame[id].h=val;
		else {
			printf("ERROR: frame%d Y end =%d, over %d\n",id+1,val+tFrame[id].y,YMAX);
			tFrame[id].status=FRAME_PARAMS_ERROR;
		}

	}
	else if (strstr(key,"alarm")!=NULL) {
		if (strstr("on",value)!=NULL)
			tFrame[id].alarm_status=1;
		else 
			tFrame[id].alarm_status=0;
	}
	else if (strstr(key,"over_temperature")!=NULL) {
		tFrame[id].over_temperature=atof(value);
	}
	else if (strstr(key,"under_temperature")!=NULL) {
		tFrame[id].under_temperature=atof(value);
	}
}
static int init_params(char *title, char *key, char *value) {
	int i=0;
	//printf("title=%s, key=%s, value=%s\n",title, key, value);
	if (strstr(title,CONNECTIVITY_TITLE)!=NULL) {
		set_conn_type(key, value);
	}
	else {
		while (i<FRAME_NUMBER) {
			//printf("frame_title=%s, title =%s\n", frame_title[i], title);
			if (strstr(frame_title[i],title)!=NULL) {
				set_frame_params(i,key,value);
			}
			i++;
		}
	}
	return 0;
}

void ini_print() {
	char *docmd;
	printf("Connection type = %s\n",ini_info_str[ini_params.conn_type]);
	printf("Post type = %s\n",ini_info_str[ini_params.post_type]);
	printf("DIO support = %d\n", ini_params.dio_support);
	printf("Alarm support = %d\n", ini_params.alarm);
	printf("Over temperature level 1 = %d\n", ini_params.over_alarm1);
	printf("Over temperature level 2 = %d\n", ini_params.over_alarm2);
	printf("Under temperature = %d\n", ini_params.under_alarm);
	docmd=dio_command_get(DO1_ON);
	print_dio_cmd(docmd);
	docmd=dio_command_get(DO2_ON);
	print_dio_cmd(docmd);
	docmd=dio_command_get(DO1_OFF);
	print_dio_cmd(docmd);
	docmd=dio_command_get(DO2_OFF);
	print_dio_cmd(docmd);
}
void ir8062_params_print() {
	int i=0;
	while (i < FRAME_NUMBER) {
		if (tFrame[i].status==FRAME_ENABLE)
			printf("FRAME%d: X=%d, Y=%d, W=%d, H=%d, alarm %d, over=%f, under=%f\n", i+1, tFrame[i].x,tFrame[i].y,tFrame[i].w,tFrame[i].h,tFrame[i].alarm_status,tFrame[i].over_temperature,tFrame[i].under_temperature);
		else if (tFrame[i].status==FRAME_PARAMS_ERROR)
			printf("FRAME%d: Wrong ini file format\n",i+1);
		else
			printf("FRAME%d disable\n",i+1);
		i++;
	}
}

unsigned char get_rs485_cmd_len() {
	return rs485_cmd_len;
}
char *dio_command_get(char fun) {
	switch (fun) {
	case DO1_ON:
		//printf("DO1 ON command pointer\n");
		if (do1_on != NULL)
			return do1_on;
		else return NULL;
	case DO2_ON:
		//printf("DO2 ON command pointer\n");
		if (do2_on != NULL)
			return do2_on;
		else return NULL;
	case DO1_OFF:
		//printf("DO1 OFF command pointer\n");
		if (do1_off != NULL)
			return do1_off;
		else return NULL;
	case DO2_OFF:
		//printf("DO2 OFF command pointer\n");
		if (do2_off != NULL)
			return do2_off;
		else return NULL;
	default:
		printf("ERROR No command pointer\n");
		return NULL;
	}
}
int parse_ini_file(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        return -1;
    }

    char line[MAX_LINE_LEN];
    char current_section[MAX_LINE_LEN] = "";
    while (fgets(line, MAX_LINE_LEN, file)) {
        // Remove leading and trailing whitespace
        char* trimmed_line = line;
        while (*trimmed_line == ' ' || *trimmed_line == '\t' || *trimmed_line == '\r' || *trimmed_line == '\n') {
            ++trimmed_line;
        }
        char* end = trimmed_line + strlen(trimmed_line) - 1;
        while (end > trimmed_line && (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')) {
            --end;
        }
        *(end + 1) = '\0';

        // Skip empty lines and comments
        if (*trimmed_line == '\0' || *trimmed_line == '#') {
            continue;
        }

        // Check for section headers
        if (*trimmed_line == '[' && *end == ']') {
            *end = '\0';
            strcpy(current_section, trimmed_line + 1);
        } else {
            // Parse key-value pairs
            char* equals_sign = strchr(trimmed_line, '=');
            if (equals_sign) {
                *equals_sign = '\0';
                char* key = trimmed_line;
                char* value = equals_sign + 1;
				init_params(current_section,key,value);
 //               printf("[%s] %s=%s\n", current_section, key, value);
            }
        }
    }
	ir8062_params_print();
	ini_print();
    fclose(file);
    return 1;
}

tFrame_t get_ini_frameinfo(int id) {
	return tFrame[id];
}
unsigned char get_ini_conn_type() {
	return ini_params.conn_type;
}

unsigned char get_ini_post_type() {
	return ini_params.post_type;
}

unsigned char get_ini_alarm() {
	return ini_params.alarm;
}
unsigned int get_ini_over_alarm1() {
	return ini_params.over_alarm1;
}
unsigned int get_ini_over_alarm2() {
	return ini_params.over_alarm2;
}

unsigned int get_ini_under_alarm() {
	return ini_params.under_alarm;
}
#if INI_PARSE_DEBUG
int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s filename.ini\n", argv[0]);
        return 1;
    }
    if (!parse_ini_file(argv[1])) {
        fprintf(stderr, "Error parsing file: %s\n", argv[1]);
        return 1;
    }
    return 0;
}
#endif

