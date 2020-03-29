#include <iostream> 
#include <fstream>
#include <vector>
#include <string>
using namespace std;


void PinyinGrep(vector<string>& GrepLines, ifstream& fin, const string& regex)
{
    string filename;
    getline(fin, filename);

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