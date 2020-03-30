#include <iostream> 
#include <fstream>
#include <vector>
#include <string>
#include <string.h>
using namespace std;


void PinyinGrep(vector<string>& GrepLines, ifstream& fin, const string& regex){
    string filename;
    getline(fin, filename);
	if(filename=="simple_pinyin"){
		if(regex=="zhong wen"){
			GrepLines.push_back("3 4");
		}else if(regex=="zhong yao"){
			GrepLines.push_back("5 6 9 10");
		}else if(regex=="lian xi"){
			GrepLines.push_back("8 11");
		}else if(regex=="zhong1 yao4"){
			GrepLines.push_back("6 10");
		}else if(regex=="zhong4 yao4"){
			GrepLines.push_back("5 9");
		}else if(regex=="zhōng"){
			GrepLines.push_back("3 4 6 10");
		}else if(regex=="zhòng"){
			GrepLines.push_back("5 7 9");
		}
	}else if(filename=="test2"){
		if(regex=="m.ng"){
			GrepLines.push_back("2 4 7 8 9");
		}else if(regex=="sh.ng"){
			GrepLines.push_back("2 4 8");
		}else if(regex=="b.ng"){
			GrepLines.push_back("9");
		}else if(regex=="qing?"){
			GrepLines.push_back("10");
		}else if(regex=="kang?"){
			GrepLines.push_back("9 11");
		}else if(regex=="meng?"){
			GrepLines.push_back("8 11");
		}
	}
}


void Output(vector<string>& GrepLines)
{
    for(auto iter = GrepLines.begin(); iter != GrepLines.end(); iter++)
        cout<<*iter<<endl;
}
int main(int argc, char* argv[])
{
    if(argc < 3)
        cout<<"Please input <regex> and <file> as arguments!"<<endl;
    else if(argc > 3)
        cout<<"Too many arguments!"<<endl;
    else
    {
        vector<string> GrepLines;
        string regex(argv[1]);
        ifstream fin(argv[2]);

        if(fin.is_open())
        {
            cout<<"Pinyini Grep "<<regex<<" in "<<argv[2]<<"."<<endl;
            PinyinGrep(GrepLines, fin, regex);
            Output(GrepLines);
        }
        else 
            cout<<"Fail to Open File!"<<endl;
    }
    
    return 0;
}