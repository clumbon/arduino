/*******************
モータライズタンク自律走行プログラム

var03	同挙動一定時間継続時に挙動変更処理実行追加
var04	旋回時後進挙動調整(旋回完了前に後進先停止)
		旋回時間ランダム化
*******************/
//測距ピン
#define ECHO_R 10
#define TRIG_R 11
#define ECHO_L 12
#define TRIG_L 13
//モーターピン
#define MR_1 8
#define MR_2 7
#define MR_PWM 9
#define ML_1 5
#define ML_2 4
#define ML_PWM 3
#define M_STBY 6
//モーター出力値(pwm調整)
#define M_PWM_H_R 160	//起動右
#define M_PWM_H_L 160	//起動左
#define M_PWM_TIME 50	//起動キープms
#define M_PWM_N_R 148	//巡行右
#define M_PWM_N_L 147	//巡行左
//乱数seedピン
#define A_SEED 0
//同一挙動時変更時間(ms調整)
#define T_INTERRUPT 6000
//回避距離(cm調整){緊急回避限度,通常回避限度}
const int DODGE_DIST[2] {3, 15};
//測距値格納
double g_duration_rl[2]={0, 0};	//{右,左}
double g_distance_rl[2]={0, 0};	//{右,左}
//障害物距離一時格納
int g_tmp[2]={0, 0};	//{右,左}0=緊急回避,1=通常回避,2=障害なし
//モーター挙動指示値格納
int g_mset_rl;	//モーター左右指定 0=右,1=左
int g_mset_fb;	//モーター回転指定 0=停止 1=正転 2=逆転
//モーター状態格納
int g_mstate[2]={0, 0};	//{右, 左}0=停止 1=正転 2=逆転
//モーター状態一時格納(継続時間確認用)
int g_mstate_tmp[2];
//モーターpin no {{右pin1,pin2,pin_pwm,起動pwm値,巡行pwm値}
//				, {左pin1,pin2,pin_pwm,起動pwm値,巡行pwm値}}
int g_mpinno[2][5]={{MR_1, MR_2, MR_PWM, M_PWM_H_R, M_PWM_N_R}
				, {ML_1, ML_2, ML_PWM, M_PWM_H_L, M_PWM_N_L}};
//モーター起動ピンセット値 {{停止},{前進},{後進}}
int mpinset[3][2]={ {0, 0}, {1, 0}, {0, 1} };
//挙動継続時間格納
unsigned long g_continuation;

void setup() {
	//測距ピンセット
	pinMode(ECHO_R, INPUT);
	pinMode(ECHO_L, INPUT);
	pinMode(TRIG_R, OUTPUT);
	pinMode(TRIG_L, OUTPUT);
	//測距準備
	digitalWrite(TRIG_R, LOW);
	digitalWrite(TRIG_L, LOW);
	//モーターピンセット
	pinMode(MR_1, OUTPUT);
	pinMode(MR_2, OUTPUT);
	pinMode(MR_PWM, OUTPUT);
	pinMode(ML_1, OUTPUT);
	pinMode(ML_2, OUTPUT);
	pinMode(ML_PWM, OUTPUT);
	pinMode(M_STBY, OUTPUT);
	digitalWrite(M_STBY, HIGH);
	g_continuation=millis()+T_INTERRUPT;
}

void loop() {
	//エコー時間取得
	g_duration_rl[0]=get_Duration(TRIG_R, ECHO_R);
	g_duration_rl[1]=get_Duration(TRIG_L, ECHO_L);
	//エコー距離変換
	g_distance_rl[0]=get_Distance(g_duration_rl[0]);
	g_distance_rl[1]=get_Distance(g_duration_rl[1]);

	//障害物限度距離チェック
	for(int t_rl=0; t_rl<2; t_rl++){
		g_tmp[t_rl]=2;
		for(int t_dis=0; t_dis<2; t_dis++){
			if(g_distance_rl[t_rl] <= DODGE_DIST[t_dis]){
				g_tmp[t_rl]=t_dis;
				break;
			}
		}
	}
	
	//回避処理実行
	avoidance_Process(g_tmp);
	delay(50);
}

/*************************
回避処理振分

左右測距値に応じて挙動処理

tmp={右,左} 0=緊急回避,1=通常回避,2=障害なし

どちらも障害無
→前進
どちらかが緊急回避距離
→停止、バック、旋回
どちらかが通常回避距離
→回避方=前進、障害無方=停止
両方が通常回避距離
→旋回(左右ランダム)

同挙動が規定時間以上継続
→停止、旋回
*************************/
void avoidance_Process(int tmp[]){
	memcpy(g_mstate_tmp, g_mstate, sizeof(g_mstate));
	randomSeed(analogRead(A_SEED));
	int rnd_rl=random(2);
	
	if(tmp[0]==2 && tmp[1]==2){
		//前進
		run_Motor(0, 1);
		run_Motor(1, 1);
	}else if(!tmp[0] || !tmp[1]){
		//停止バック旋回
		run_Motor(0, 0);
		run_Motor(1, 0);
		delay(500);
		run_Motor(0, 2);
		run_Motor(1, 2);
		delay(800);
		run_Motor(rnd_rl, 1);
		run_Motor(abs(rnd_rl-1), 2);
		delay(random(500)+800);
		run_Motor(abs(rnd_rl-1), 0);
		delay(100);
	}else if(tmp[0] != tmp[1]){
		//片側回避
		if(tmp[0]==1){
			run_Motor(1, 0);
			run_Motor(0, 1);
		}else{
			run_Motor(0, 0);
			run_Motor(1, 1);
		}
	}else{
		//旋回
		run_Motor(rnd_rl, 1);
		run_Motor(abs(rnd_rl-1), 2);
		delay(random(500)+600);
		run_Motor(abs(rnd_rl-1), 0);
		delay(100);
	}
	
	if(memcmp(g_mstate, g_mstate_tmp, sizeof(g_mstate))){
		g_continuation=millis()+T_INTERRUPT;
	}else if(g_continuation < millis()){
		run_Motor(0, 0);
		run_Motor(1, 0);
		delay(100);
		if(!random(5)){
			run_Motor(0, 2);
			run_Motor(1, 2);
			delay(500);
		}
		run_Motor(rnd_rl, 1);
		run_Motor(abs(rnd_rl-1), 2);
		delay(random(500)+200);
		run_Motor(0, 0);
		run_Motor(1, 0);
		g_continuation=millis()+T_INTERRUPT;
	}
}

/*************************
モーター回転

rl 0=右モーター,1=左モーター
fb 0=停止,1=前進,2=後退

安定迄起動パルス送信後、巡行パルスセット
既に同処理実行中ならreturn
*************************/
void run_Motor(int rl, int fb){
	if(g_mstate[rl]==fb)return;

	if(fb==0){
		analogWrite(g_mpinno[rl][2], 0);
		digitalWrite(g_mpinno[rl][0], LOW);
		digitalWrite(g_mpinno[rl][1], LOW);
		g_mstate[rl]=fb;
		return;
	}

	digitalWrite(g_mpinno[rl][0], mpinset[fb][0]? HIGH: LOW);
	digitalWrite(g_mpinno[rl][1], mpinset[fb][1]? HIGH: LOW);
	analogWrite(g_mpinno[rl][2], g_mpinno[rl][3]);
	delay(M_PWM_TIME);
	analogWrite(g_mpinno[rl][2], g_mpinno[rl][4]);
	g_mstate[rl]=fb;
}
/*************************
測距値取得
*************************/
double get_Duration(int trig, int echo){
	double duration;
	digitalWrite(trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig, LOW);
	duration=pulseIn(echo, HIGH, 20000);
	if(duration==0)duration=20000;
	return duration;
}
/*************************
測距値距離変換
*************************/
double get_Distance(double duration){
	double distance=0;
	/*
	//duration/=2;
	//distance=duration*340*100/1000000;
	*/
	distance=duration * 0.017;
	return distance;
}
