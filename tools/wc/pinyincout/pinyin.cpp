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
            multiCheck_store.push_back(temp);
            to_check = trim(to_check);
        }
        if(to_check!="")
        {
            multiCheck_store.push_back(to_check);
        }
    }
}