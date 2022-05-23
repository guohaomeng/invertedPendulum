#include <Arduino.h>
//回调函数指针定义
typedef void (*CommandCallback)(char *); //!< 命令回调函数指针
class Command
{
public:
  void add(char *id, CommandCallback onCommand);
  void run(char *str);
  void scalar(float *value, char *user_cmd);
  bool isSentinel(char *ch, char *str);
private:
  //订阅的命令回调变量
  CommandCallback call_list[20]; //!< 命令回调指针数组-20是任意数
  char *call_ids[20];            //!< 添加了回调命令
  int call_count;                //!< 订阅的回调次数
};

void Command::run(char *str)
{
  for (int i = 0; i < call_count; i++)
  {
    if (isSentinel(call_ids[i], str))
    {                                          // case :   call_ids = "T2"   str = "T215.15"
      call_list[i](str + strlen(call_ids[i])); // get 15.15  input function
      break;
    }
  }
}
void Command::add(char *id, CommandCallback onCommand)
{
  call_list[call_count] = onCommand;
  call_ids[call_count] = id;
  call_count++;
}
void Command::scalar(float *value, char *user_cmd)
{
  *value = atof(user_cmd);
  Serial.printf("%s\n",user_cmd);
}
bool Command::isSentinel(char *ch, char *str)
{
  char s[strlen(ch) + 1];
  strncpy(s, str, strlen(ch));
  s[strlen(ch)] = '\0'; // strncpy need add end '\0'
  if (strcmp(ch, s) == 0)
    return true;
  else
    return false;
}

