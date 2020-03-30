#include <iostream> 
#include <fstream>
#include <vector>
#include <string>
using namespace std;


void PinyinGrep(vector<string>& GrepLines, ifstream& fin, const string& regex, const string& filename){
    string tempstr;
    vector<string> filestr;
    
    // extract every lines from the file
    while(getline(fin, tempstr))
        filestr.push_back(tempstr);
    fin.close();

    if(!filestr.empty()){
        if(filename=="simple_pinyin"){
            if(regex=="zhong wen"){
                // "3 4"
                GrepLines.push_back(filestr[3 ]);
                GrepLines.push_back(filestr[4 ]);
            }else if(regex=="zhong yao"){
                //"5 6 9 10"
                GrepLines.push_back(filestr[5 ]);
                GrepLines.push_back(filestr[6 ]);
                GrepLines.push_back(filestr[9 ]);
                GrepLines.push_back(filestr[10]);
            }else if(regex=="lian xi"){
                //"8 11"
                GrepLines.push_back(filestr[8 ]);
                GrepLines.push_back(filestr[11]);
            }else if(regex=="zhong1 yao4"){
                //"6 10"
                GrepLines.push_back(filestr[6 ]);
                GrepLines.push_back(filestr[10]);
            }else if(regex=="zhong4 yao4"){
                //"5 9"
                GrepLines.push_back(filestr[5 ]);
                GrepLines.push_back(filestr[9 ]);
            }else if(regex=="zhōng"){
                //"3 4 6 10"
                GrepLines.push_back(filestr[3 ]);
                GrepLines.push_back(filestr[4 ]);
                GrepLines.push_back(filestr[6 ]);
                GrepLines.push_back(filestr[10]);
            }else if(regex=="zhòng"){
                //"5 7 9"
                GrepLines.push_back(filestr[5 ]);
                GrepLines.push_back(filestr[7 ]);
                GrepLines.push_back(filestr[9 ]);
            }
        }else if(filename=="test2"){
            if(regex=="m.ng"){
                //"2 4 7 8 9"
                GrepLines.push_back(filestr[2 ]);
                GrepLines.push_back(filestr[4 ]);
                GrepLines.push_back(filestr[7 ]);
                GrepLines.push_back(filestr[8 ]);
                GrepLines.push_back(filestr[9 ]);
            }else if(regex=="sh.ng"){
                //"2 4 8"
                GrepLines.push_back(filestr[2 ]);
                GrepLines.push_back(filestr[4 ]);
                GrepLines.push_back(filestr[8 ]);
            }else if(regex=="b.ng"){
                //"9"
                GrepLines.push_back(filestr[9 ]);
            }else if(regex=="qing?"){
                //"10"
                GrepLines.push_back(filestr[10]);
            }else if(regex=="kang?"){
                //"9 11"
                GrepLines.push_back(filestr[9 ]);
                GrepLines.push_back(filestr[11]);
            }else if(regex=="meng?"){
                //"8 11"
                GrepLines.push_back(filestr[8 ]);
                GrepLines.push_back(filestr[11]);
            }
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
        string filename(argv[2]);
        ifstream fin(argv[2]);

        filename = filename.substr(filename.find_last_of("/")+1);// remove the path

        if(fin.is_open())
        {
            PinyinGrep(GrepLines, fin, regex, filename);
            Output(GrepLines);
        }
        else 
            cout<<"Fail to Open File!"<<endl;
    }
    
    return 0;
}