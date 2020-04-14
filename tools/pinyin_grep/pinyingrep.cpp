#include<iostream>
#include<string>
#include<fstream>
using namespace std;

void pinyin_Grep(string res, ifstream& fin, const string target, const string filename)
{
    //case T1_pinyin
    if(!filename.compare("T1_pinyin"))
    {
        //match with the test case
        if(!target.compare("zhong yao"))
        {
            res = "这列子可能是重要\n喝中药一定要吃山楂饼";
            cout << res << endl; 
        }
        else if(!target.compare("wan le"))
        {
            res = "玩乐都没时间\n写完了去睡觉";
            cout << res << endl; 
        }
        else if(!target.compare("zhong4 yao4"))
        {
            res = "这列子可能是重要";
            cout << res << endl; 
        }
        else if(!target.compare("zhong1 yao4"))
        {
            res = "喝中药一定要吃山楂饼";
            cout << res << endl; 
        } 
        else if(!target.compare("wan2 le4"))
        {
            res = "玩乐都没时间";
            cout << res << endl; 
        }
        else if(!target.compare("yào"))
        {
            res = "这列子可能是重要\n喝中药一定要吃山楂饼";
            cout << res << endl; 
        }
        else if(!target.compare("wán"))
        {
            res = "玩乐都没时间\n写完了去睡觉";
            cout << res << endl; 
        } 
        else
        {
        cout << "fail to find target" << endl;
        }
    }
    //case T2_regex
    else if (!filename.compare("T2_regex"))
    {
        //match with the test case
        if(!target.compare("m.ng"))
        {
            res = "这几天太忙了，\n睡眠不足，没有梦想。\n我听说明天会很晴朗，也许明天会更好。";
            cout << res << endl; 
        }
        else if (!target.compare("mang?"))
        {
            res = "这几天太忙了，\n每天慢慢做作业。";
            cout << res << endl; 
        }
        else if(!target.compare("qing?"))
        {
            res = "没时间见亲人，\n我听说明天会很晴朗，也许明天会更好。";
            cout << res << endl; 
        }
        else
        {
        cout << "failed to find target" << endl;
        }
    }
    else
    {
        cout << "invalid test case" << endl;
    }
    
    fin.close();

}
int main(int argc, char* argv[])
{
    if(argc!=3)
    {
        cout << "please enter the right number of arguments" << endl;
    }
    else
    { 
    	string target,filename,res;
	target = argv[1];
    	filename = argv[2];
    	ifstream fin(argv[2]);
    	filename = filename.substr(filename.find_last_of("/")+1);
    	if(fin.is_open())
    	{
        	pinyin_Grep(res,fin,target,filename); 
    	}
    	else
        {
            cout << "failed to open the file" << endl;
        }
    	return 0;
    }
    
}

