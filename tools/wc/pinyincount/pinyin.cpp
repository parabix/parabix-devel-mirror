#include"pinyin.h"
namespace PinyinPattern{
    void InputChecker :: MultiInput_Check(string to_check, vector<string> &multiCheck_store)
    {
        int pos = 0;
        to_check = trim(to_check);
        while(pos!=-1)
        {
            string temp;
            pos = to_check.find_first_of(' ',pos);
            temp = to_check.substr(0,pos);
            to_check.erase(0,pos);
            temp = trim(temp);
            multiCheck_store.push_back(temp);
            to_check = trim(to_check);
        }
        if(to_check!="")
        {
            multiCheck_store.push_back(to_check);
        }
    }
    //for now the check is only availble for digital tone
    void InputChecker :: Divide()
    {
        for(int i = 0;i!=first_check.size();i++)
        {
            //if it is tag with digital tone

            string temp;
            pair <string,int> temp_pair;
            temp = first_check[i];
            if(temp.substr(temp.size()-1,1)=="1")
            {
                temp_pair = make_pair(temp.substr(0,temp.size()-1),1);
            }
            else if(temp.substr(temp.size()-1,1)=="2")
            {
                temp_pair = make_pair(temp.substr(0,temp.size()-1),1);
            }
            else if(temp.substr(temp.size()-1,1)=="3")
            {
                temp_pair = make_pair(temp.substr(0,temp.size()-1),1);
            }
            else if(temp.substr(temp.size()-1,1)=="4")
            {
                temp_pair = make_pair(temp.substr(0,temp.size()-1),1);
            }
            //default
            else
            {
                cout << "not available yet"<<endl;
            }
            final.push_back(temp_pair);
        }
    }
    //the next part should match .h database with the divided pairs
}