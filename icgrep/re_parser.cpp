/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_parser.h"

RE_Parser::RE_Parser(){}

ParseResult* RE_Parser::parse_re(std::string input_string)
{
    parse_result_retVal re_result = parse_re_helper(input_string);

    if (re_result.remaining.length() == 0)
    {
        return re_result.result;
    }
    else if (ParseSuccess* re_success = dynamic_cast<ParseSuccess*>(re_result.result))
    {
        ParseFailure* failure = new ParseFailure("Junk remaining!");

        return failure;
    }
    else if (ParseFailure* re_failure = dynamic_cast<ParseFailure*>(re_result.result))
    {
        return re_failure;
    }
    else
    {
        return 0;
    }
}

parse_result_retVal RE_Parser::parse_re_helper(std::string s)
{
    parse_result_retVal result_retVal;

    parse_re_list_retVal af_result = parse_re_alt_form_list(s);

    if (af_result.re_list.size() == 0)
    {
        result_retVal.result = new ParseFailure("No regular expression found!");
        result_retVal.remaining = s;
    }
    else if (af_result.re_list.size() == 1)
    {
        result_retVal.result = new ParseSuccess(af_result.re_list.front());
        result_retVal.remaining = af_result.remaining;
    }
    else
    {
        result_retVal.result = new ParseSuccess(new Alt(&af_result.re_list));
        result_retVal.remaining = af_result.remaining;
    }

    return result_retVal;
}

parse_re_list_retVal RE_Parser::parse_re_alt_form_list(std::string s)
{
    parse_re_list_retVal re_list_retVal;
    parse_result_retVal form_result = parse_re_form(s);

    if (ParseSuccess* re_success = dynamic_cast<ParseSuccess*>(form_result.result))
    {
        if (form_result.remaining.operator [](0) == '|')
        {
            parse_re_list_retVal t1_re_list_retVal =
                    parse_re_alt_form_list(form_result.remaining.substr(1, form_result.remaining.length() - 1));

            std::list<RE*>::iterator it;
            it=t1_re_list_retVal.re_list.begin();
            re_list_retVal.re_list.assign(it, t1_re_list_retVal.re_list.end());
            re_list_retVal.re_list.push_back(re_success->getRE());
            re_list_retVal.remaining = t1_re_list_retVal.remaining;
        }
        else
        {
            re_list_retVal.remaining = form_result.remaining;
        }
        re_list_retVal.re_list.push_back(re_success->getRE());
    }
    else
    {
        re_list_retVal.re_list.clear();
        re_list_retVal.remaining = s;
    }

    return re_list_retVal;
}

parse_result_retVal RE_Parser::parse_re_form(std::string s)
{
    parse_result_retVal form_retVal;
    parse_re_list_retVal item_list_result = parse_re_item_list(s);

    if (item_list_result.re_list.size() == 0)
    {
        form_retVal.result = new ParseFailure("No Regular Expression Found!");
        form_retVal.remaining = s;
    }
    else if (item_list_result.re_list.size() == 1)
    {
        form_retVal.result = new ParseSuccess(item_list_result.re_list.front());
        form_retVal.remaining = item_list_result.remaining;
    }
    else
    {
        form_retVal.result = new ParseSuccess(new Seq(&item_list_result.re_list));
        form_retVal.remaining = item_list_result.remaining;
    }

    return form_retVal;
}

parse_re_list_retVal RE_Parser::parse_re_item_list(std::string s)
{
    parse_re_list_retVal item_list_retVal;
    parse_result_retVal item_result = parse_re_item(s);

    if (ParseSuccess* re_success = dynamic_cast<ParseSuccess*>(item_result.result))
    {
        parse_re_list_retVal t1_item_list_retVal = parse_re_item_list(item_result.remaining);

        std::list<RE*>::iterator it;
        it=t1_item_list_retVal.re_list.begin();
        item_list_retVal.re_list.assign(it, t1_item_list_retVal.re_list.end());
        item_list_retVal.re_list.push_back(re_success->getRE());
        item_list_retVal.remaining = t1_item_list_retVal.remaining;
    }
    else
    {
        item_list_retVal.re_list.clear();
        item_list_retVal.remaining = s;
    }

    return item_list_retVal;
}

parse_result_retVal RE_Parser::parse_re_item(std::string s)
{
    parse_result_retVal item_retVal;
    parse_result_retVal unit_result = parse_re_unit(s);

    if (ParseSuccess* re_success = dynamic_cast<ParseSuccess*>(unit_result.result))
    {
        item_retVal = extend_item(re_success->getRE(), unit_result.remaining);
    }
    else
    {
        item_retVal.result = new ParseFailure("Parse item failure!");
        item_retVal.remaining = s;
    }

    return item_retVal;
}

parse_result_retVal RE_Parser::parse_re_unit(std::string s)
{
    parse_result_retVal unit_retVal;

    if (s.length() == 0)
    {
        unit_retVal.result = new ParseFailure("Incomplete regular expression! (parse_re_unit)");
        unit_retVal.remaining = "";
    }
    else if (s.operator[](0) == '(')
    {
        parse_result_retVal t1_unit_retVal = parse_re_helper(s.substr(1, s.length() - 1));
        ParseSuccess* success = dynamic_cast<ParseSuccess*>(t1_unit_retVal.result);
        if ((success != 0) && (t1_unit_retVal.remaining.operator[](0) == ')'))
        {
            unit_retVal.result = success;
            unit_retVal.remaining = t1_unit_retVal.remaining.substr(1, t1_unit_retVal.remaining.length() - 1);
        }
        else
        {
            unit_retVal.result = new ParseFailure("Bad parenthesized RE!");
            unit_retVal.remaining = s.substr(1, s.length() - 1);
        }
    }
    else if (s.operator [](0) == '^')
    {
        unit_retVal.result = new ParseSuccess(new Start);
        unit_retVal.remaining = s.substr(1, s.length() - 1);
    }
    else if (s.operator[](0) == '$')
    {
        unit_retVal.result = new ParseSuccess(new End);
        unit_retVal.remaining = s.substr(1, s.length() - 1);
    }
    else
    {
        unit_retVal = parse_cc(s);
    }

    return unit_retVal;
}

parse_result_retVal RE_Parser::extend_item(RE *re, std::string s)
{
     parse_result_retVal extend_item_retVal;

     if (s.operator [](0) == '*')
     {
         return extend_item(new Rep(re, 0, new Unbounded), s.substr(1, s.length() - 1));
     }
     else if (s.operator[](0) == '?')
     {
         return extend_item(new Rep(re, 0, new UpperBound(1)), s.substr(1, s.length() - 1));
     }
     else if (s.operator[](0) == '+')
     {
         return extend_item(new Rep(re, 1, new Unbounded), s.substr(1, s.length() - 1));
     }
     else if (s.operator[](0) == '{')
     {
        parse_int_retVal int_retVal = parse_int(s.substr(1, s.length() - 1));

        if ((int_retVal.i != -1) && (int_retVal.remaining.operator [](0) == '}'))
        {
            extend_item_retVal =
                    extend_item(new Rep(re, int_retVal.i, new UpperBound(int_retVal.i)), int_retVal.remaining.substr(1, int_retVal.remaining.length() - 1));

        }
        else if ((int_retVal.i != -1) && ((int_retVal.remaining.operator [](0) == ',') && (int_retVal.remaining.operator [](1) == '}')))
        {
            extend_item_retVal =
                    extend_item(new Rep(re, int_retVal.i, new Unbounded), int_retVal.remaining.substr(2, int_retVal.remaining.length() - 2));

        }
        else if ((int_retVal.i != -1) && (int_retVal.remaining.operator [](0) == ','))
        {
            parse_int_retVal t1_int_retVal = parse_int(int_retVal.remaining.substr(1, int_retVal.remaining.length() - 1));

            if ((t1_int_retVal.i != -1) && (t1_int_retVal.remaining.operator [](0) == '}'))
            {
                extend_item_retVal =
                        extend_item(new Rep(re, int_retVal.i, new UpperBound(t1_int_retVal.i)), t1_int_retVal.remaining.substr(1, t1_int_retVal.remaining.length() - 1));
            }
            else
            {
                extend_item_retVal.result = new ParseFailure("Bad upper bound!");
                extend_item_retVal.remaining = int_retVal.remaining.substr(1, int_retVal.remaining.length() - 1);
            }
        }
        else
        {
            extend_item_retVal.result = new ParseFailure("Bad lower bound!");
            extend_item_retVal.remaining = s.substr(1, s.length() - 1);
        }
     }
     else
     {
         extend_item_retVal.result = new ParseSuccess(re);
         extend_item_retVal.remaining = s;
     }

     return extend_item_retVal;
}

parse_result_retVal RE_Parser::parse_cc(std::string s)
{
    parse_result_retVal cc_retVal;

    if (s.operator [](0) == '\\')
    {
        if (s.operator [](1) == '?')
        {
            cc_retVal.result = new ParseSuccess(new CC('?'));
        }
        else if (s.operator [](1) == '+')
        {
            cc_retVal.result = new ParseSuccess(new CC('+'));
        }
        else if (s.operator [](1) == '*')
        {
            cc_retVal.result = new ParseSuccess(new CC('*'));
        }
        else if (s.operator [](1) == '(')
        {
            cc_retVal.result = new ParseSuccess(new CC('('));
        }
        else if (s.operator [](1) == ')')
        {
            cc_retVal.result = new ParseSuccess(new CC(')'));
        }
        else if (s.operator [](1) == '{')
        {
            cc_retVal.result = new ParseSuccess(new CC('{'));
        }
        else if (s.operator [](1) == '}')
        {
            cc_retVal.result = new ParseSuccess(new CC('}'));
        }
        else if (s.operator [](1) == '[')
        {
            cc_retVal.result = new ParseSuccess(new CC('['));
        }
        else if (s.operator [](1) == ']')
        {
            cc_retVal.result = new ParseSuccess(new CC(']'));
        }
        else if (s.operator [](1) == '|')
        {
            cc_retVal.result = new ParseSuccess(new CC('|'));
        }
        else if (s.operator [](1) == '.')
        {
            cc_retVal.result = new ParseSuccess(new CC('.'));
        }
        else if (s.operator [](1) == '\\')
        {
            cc_retVal.result = new ParseSuccess(new CC('\\'));
        }
        else if (s.operator [](1) == 'u')
        {
            parse_int_retVal hex_val = parse_hex(s.substr(2, s.length() - 2));

            if (hex_val.i == -1)
            {
                cc_retVal.result = new ParseFailure("Bad Unicode hex notation!");
                cc_retVal.remaining = hex_val.remaining;
            }
            else
            {
                cc_retVal.result = new ParseSuccess(new CC(hex_val.i));
                cc_retVal.remaining = hex_val.remaining;
            }

            return cc_retVal;
        }
        else
        {
            cc_retVal.result = new ParseFailure("Illegal backslash escape!");
            cc_retVal.remaining = s.substr(1, s.length() - 1);

            return cc_retVal;
        }

        cc_retVal.remaining = s.substr(2, s.length() - 2);

        return cc_retVal;
    }

    if (s.operator [](0) == '.')
    {        
        CC* cc = new CC();
        cc->insert_range(0, 9);
        cc->insert_range(11, 127);
        cc_retVal.result = new ParseSuccess(cc);
        cc_retVal.remaining = s.substr(1, s.length() - 1);

        return cc_retVal;
    }

    if (s.operator [](0) == '[')
    {
        if (s.operator[](1) == '^')
        {
            cc_retVal = negate_cc_result(parse_cc_body(s.substr(2, s.length() - 2)));
        }
        else
        {
            cc_retVal = parse_cc_body(s.substr(1, s.length() - 1));
        }

        return cc_retVal;
    }

    std::string metacharacters = "?+*(){}[]|";
    std::string c = s.substr(0,1);

    if (metacharacters.find(c) == std::string::npos)
    {
        cc_retVal.result = new ParseSuccess(new CC(s.operator [](0)));
        cc_retVal.remaining = s.substr(1, s.length() - 1);
    }
    else
    {
        cc_retVal.result = new ParseFailure("Metacharacter alone!");
        cc_retVal.remaining = s;
    }

    return cc_retVal;
}

parse_result_retVal RE_Parser::parse_cc_body(std::string s)
{
    parse_result_retVal result_retVal;

    if (s.length() == 0)
    {
        result_retVal.result = new ParseFailure("Unclosed character class!");
        result_retVal.remaining = "";
    }
    else
    {
        CC* cc = new CC();
        result_retVal = parse_cc_body1(s.operator [](0), s.substr(1, s.length() - 1), cc);
    }

    return result_retVal;
}

parse_result_retVal RE_Parser::parse_cc_body0(std::string s, CC* cc_sofar)
{
    parse_result_retVal cc_body0_retVal;

    if (s.length() == 0)
    {
        delete cc_sofar;
        cc_body0_retVal.result = new ParseFailure("Unclosed character class!");
        cc_body0_retVal.remaining = "";
    }
    else if (s.operator [](0) == ']')
    {
        cc_body0_retVal.result = new ParseSuccess(cc_sofar);
        cc_body0_retVal.remaining = s.substr(1, s.length() - 1);
    }
    else if ((s.operator [](0) == '-') && (s.operator [](1) == ']'))
    {
        cc_sofar->insert1('-');
        cc_body0_retVal.result = new ParseSuccess(cc_sofar);
        cc_body0_retVal.remaining = s.substr(2, s.length() - 2);
    }
    else if (s.operator [](0) == '-')
    {
        delete cc_sofar;
        cc_body0_retVal.result = new ParseFailure("Bad range in character class!");
        cc_body0_retVal.remaining = s.substr(1, s.length() - 1);
    }
    else
    {
        cc_body0_retVal = parse_cc_body1(s.operator [](0), s.substr(1, s.length() - 1), cc_sofar);
    }

    return cc_body0_retVal;
}

parse_result_retVal RE_Parser::parse_cc_body1(int chr, std::string s, CC* cc_sofar)
{
    parse_result_retVal cc_body1_retVal;

    if (s.length() == 0)
    {
        delete cc_sofar;
        cc_body1_retVal.result = new ParseFailure("Unclosed character class!");
        cc_body1_retVal.remaining = "";
    }
    else if (s.operator [](0) == ']')
    {
        cc_sofar->insert1(chr);
        cc_body1_retVal.result = new ParseSuccess(cc_sofar);
        cc_body1_retVal.remaining = s.substr(1, s.length() - 1);
    }
    else if (s.length() == 1)
    {
        delete cc_sofar;
        cc_body1_retVal.result = new ParseFailure("Unclosed character class!");
        cc_body1_retVal.remaining = "";
    }
    else if ((s.operator [](0) == '-') && (s.operator [](1) == ']'))
    {
        cc_sofar->insert1(chr);
        cc_sofar->insert1('-');
        cc_body1_retVal = parse_cc_body0(s, cc_sofar);
    }
    else if ((s.operator [](0) == '-') && (s.operator [](1) == '\\') && (s.operator [](2) == 'u'))
    {
        parse_int_retVal int_retVal = parse_hex(s.substr(3, s.length() - 3));

        if (int_retVal.i == -1)
        {
            cc_body1_retVal.result = new ParseFailure("Bad Unicode hex notation!");
            cc_body1_retVal.remaining = "";
        }
        else
        {
            cc_sofar->insert_range(chr, int_retVal.i);
            cc_body1_retVal = parse_cc_body0(int_retVal.remaining, cc_sofar);
        }
    }
    else if ((s.operator [](0) == '-') && ( s.length() > 1))
    {
        cc_sofar->insert_range(chr, s.operator [](1));
        cc_body1_retVal = parse_cc_body0(s.substr(2, s.length() - 2), cc_sofar);
    }
    else if ((s.operator [](0) == 'u') && ( s.length() > 1))
    {
        parse_int_retVal int_retVal = parse_hex(s.substr(1, s.length() - 1));

        if (int_retVal.i == -1)
        {
            cc_body1_retVal.result = new ParseFailure("Bad Unicode hex notation!");
            cc_body1_retVal.remaining = "";
        }
        else
        {
            cc_body1_retVal = parse_cc_body1(int_retVal.i, int_retVal.remaining, cc_sofar);
        }
    }
    else
    {
        cc_sofar->insert1(chr);
        cc_body1_retVal = parse_cc_body1(s.operator [](0), s.substr(1, s.length() - 1), cc_sofar);
    }

    return cc_body1_retVal;
}

parse_int_retVal RE_Parser::parse_hex(std::string s)
{
    parse_int_retVal int_retVal;

    if (s.operator [](0) == '{')
    {
        int hexval_sofar = 0;
        int_retVal = parse_hex_body(hexval_sofar, s.substr(1, s.length() - 1));

    }
    else
    {
        int_retVal.i = -1;
        int_retVal.remaining = s;
    }

    return int_retVal;
}

parse_int_retVal RE_Parser::parse_hex_body(int i, std::string s)
{
    parse_int_retVal int_retVal;

    if (s.length() == 0)
    {
        int_retVal.i = i;
        int_retVal.remaining = "";
    }
    else if (s.operator [](0) == '}')
    {
        int_retVal.i = i;
        int_retVal.remaining = s.substr(1, s.length() - 1);
    }
    else if ((s.operator [](0) >= '0') && (s.operator [](0) <= '9'))
    {
        int_retVal = parse_hex_body(parse_hex_body1(i, s.substr(0,1)), s.substr(1, s.length() - 1));
    }
    else if ((s.operator [](0) >= 'a') && (s.operator [](0) <= 'f'))
    {
        int_retVal = parse_hex_body(parse_hex_body1(i, s.substr(0,1)), s.substr(1, s.length() - 1));
    }
    else if ((s.operator [](0) >= 'A') && (s.operator [](0) <= 'F'))
    {
        int_retVal = parse_hex_body(parse_hex_body1(i, s.substr(0,1)), s.substr(1, s.length() - 1));
    }
    else
    {
        int_retVal.i = -1;
        int_retVal.remaining = s;
    }

    return int_retVal;
}

int RE_Parser::parse_hex_body1(int i, std::string hex_str)
{
    int retVal = 0;
    int newVal = 0;

    retVal = i << 4;

    std::stringstream ss(hex_str);
    ss >> std::hex >> newVal;

    retVal = retVal | newVal;

    return retVal;
}

parse_int_retVal RE_Parser::parse_int(std::string s)
{
    parse_int_retVal int_retVal;

    if (isdigit(s.operator [](0)))
    {
        int_retVal = parse_int1(s.operator [](0) - 48, s.substr(1, s.length() - 1));
    }
    else
    {
        int_retVal.i = -1;
        int_retVal.remaining = s;
    }

    return int_retVal;
}

parse_int_retVal RE_Parser::parse_int1(int i, std::string s)
{
    parse_int_retVal int1_retVal;

    if (s.length() == 0)
    {
        int1_retVal.i = i;
        int1_retVal.remaining = "";
    }
    else if (isdigit(s.operator [](0)))
    {
        int1_retVal = parse_int1(i * 10 + (s.operator [](0) - 48), s.substr(1, s.length() - 1));
    }
    else
    {
        int1_retVal.i = i;
        int1_retVal.remaining = s;
    }

    return int1_retVal;
}

parse_result_retVal RE_Parser::negate_cc_result(parse_result_retVal cc_result)
{
    if (ParseSuccess* success = dynamic_cast<ParseSuccess*>(cc_result.result))
    {
        if (CC* cc = dynamic_cast<CC*>(success->getRE()))
        {
            cc->negate_class();
            //Remove any new-line.
            cc->remove1(10);
        }
    }

    return cc_result;
}























