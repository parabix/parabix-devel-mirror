#include <iostream>
#include <string>
using namespace std;

//input pairs of hexidecimal values spearate by space
//ouput bit stream

string a[] ={"0000","1000","0100","1100","0010","1010","0110","1110","0001","1001"};
string b[] ={"0101","1101","0011","1011","0111","1111"};
int main()
{
    cout<<"begin"<<endl;
    string s;
    while(getline(cin,s)){
        string v="";
        for(int len=s.length()-1;len>=0;){
            if(s[len]==' '){
                len-=1;
                continue;
            }
            if(s[len]>='a' && s[len]<='f'){
                v+= b[s[len]-'a'];
            }
            else
            {
                v+= a[s[len]-'0'];
            }
            if(s[len-1]>='a' && s[len-1]<='f'){
                v+= b[s[len-1]-'a'];
            }
            else
            {
                v+= a[s[len-1]-'0'];
            }
            
            len-=2;
            
        }
        cout<<v<<endl;

    }
}