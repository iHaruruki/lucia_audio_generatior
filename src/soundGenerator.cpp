#include <robot/soundPlayer.h>
#include <robot/speechDriver.h>
#include "soundGenerator.h"

void soundGenerator::init(int argc, char **argv)
{
    // set port name
    m_verbose=1;
    string pname_cmd="/soundGenerator/command:i";
    string pname_sta="/soundGenerator/state:o";

    // open port
    m_p_cmd.open(pname_cmd.c_str());
    m_p_sta.open(pname_sta.c_str());

    // configure
    ResourceFinder rf;
    if(m_verbose) rf.setVerbose();
    rf.setDefaultConfigFile("soundGenerator.ini");
    rf.setDefaultContext("soundGenerator");
    rf.configure(argc,argv);

    // set parameter
    m_file_init=rf.findFile("soundGenerator.ini").c_str();
    m_file_sound=rf.find("file_sound").asString();
    cout << "||| init file=" << m_file_init << endl;
    cout << "||| sound file=" << m_file_sound << endl;
    if(m_file_init.empty()){
        cout << "||| [not found] init file" << endl;
        ::exit(0);
    }
    setParameter();
}

void soundGenerator::setParameter()
{
    // read sound sources
    m_source.readTables(m_file_sound);

    // register sound data
    for(int s=0;s<m_source.size();s++){
        for(int i=0;i<m_source.dataSize(s);i++){
            // register sound data
            m_player.setSource(m_source.dirWork(s)+m_source.data(s,i,0));
            cout << "||| ----------" << endl;
            cout << "||| id=" << i << endl;
            cout << "||| file=" << m_source.data(s,i,0) << endl;
            cout << "||| text=" << m_source.data(s,i,1) << endl;
            cout << "||| label=" << m_source.data(s,i,2) << endl;
            cout << "||| ----------" << endl;
        }
    }

    // register wave files
    m_player.registerSources();
}

void soundGenerator::run()
{
    while(1){
        Bottle *btl_cmd = m_p_cmd.read(false);
        // input port
        if(btl_cmd!=NULL){
            if(btl_cmd->get(0).asString()!="#"){
                // normal play
                cout << "||| [PORT] command received" << endl;
                string label=btl_cmd->get(0).asString();
                searchFile(label);

                // output port
                cout << "||| [PORT] state sent" << endl;
                Bottle &btl_sta=m_p_sta.prepare();
                btl_sta.clear();
                btl_sta.addString(label);
                m_p_sta.write();
            }else{
                // command
                string cmd=btl_cmd->get(1).asString();
                cout << "||| [PORT] command received=" << cmd << endl;
                if(cmd=="generate"){
                    string text=btl_cmd->get(2).asString();
                    string file="tmp.wav";
                    string dir=m_source.dirWork(0);
                    string path=dir+file;
                    cout << "||| [GENERATE] path=" << path << endl;
                    ResourceFinder rf;
                    utility::generateWaveFile(rf,path,text);
                }else if(cmd=="init"){
                    // init
                    cout << "||| [INIT]" << endl;
                    m_player.deleteSource();
                    setParameter();
                    // send source
                    Bottle b;
                    m_source.writeTo(b);
                    Bottle &btl_src=m_p_sta.prepare();
                    btl_src.clear();
                    btl_src.addString("#");
                    btl_src.addString("table");
                    btl_src.addList()=b;
cout << "||| [PORT] send: " << btl_src.toString() << endl;
                    m_p_sta.write();
                }
            }
        }
        Time::delay(0.05);
    }
}

int soundGenerator::searchFile(string text)
{
    string file="";
    int cnt=0;
    int ret=-1;
    for(int s=0;s<m_source.size();s++){
        for(int i=0;i<m_source.dataSize(s);i++){
            if(text==m_source.data(s,i,2)){
                file=m_source.data(s,i,0);
                cout << "||| [PLAY] " << text << endl;
                m_player.play(cnt);
                ret=0;
                break;
            }
            cnt++;
        }
    }
    return ret;
}
