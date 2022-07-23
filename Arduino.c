/*单片机程序代码*/
const int analogInPin1 = A2; // 模拟输入引脚
const int analogInPin2 = A1; // 模拟输入引脚
const int analogInPin3 = A3; // 模拟输入引脚
const int analogInPin4 = A4; // 模拟输入引脚
int a1; //电位器电压值
int b1; //电位器电压值
int a2; //电位器电压值
int b2; //电位器电压值
void setup()
{// 初始化串口参数
Serial.begin(9600);
}
void loop() {
// 读取模拟量值下面顺序为传感器在面包板上的排序
a1 = analogRead(analogInPin1);
a2 = analogRead(analogInPin2);
b1 = analogRead(analogInPin3);
天津大学2015 届本科生毕业论文
25
b2 = analogRead(analogInPin4);
a2=a1+a2;
if (a2>1023)
{a2=a2-1023;}
if (a2<-1023)
{a2=a2+1023;}
// 打印结果到串口监视器
Serial.print(a1); //弯曲圆心角1
Serial.print('y');
Serial.print(a2); //弯曲圆心角2
Serial.print('x');
Serial.print(b1); //弯曲方向角1
Serial.print('w');
Serial.println(b2);
Serial.print('e'); //弯曲方向角2 下关节为1，上关节为2
}