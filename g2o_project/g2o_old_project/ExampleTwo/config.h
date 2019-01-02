//
// Created by tqw on 18-7-27.
//

#ifndef RGB_D_VO_ORIGIN_CONFIG_H
#define RGB_D_VO_ORIGIN_CONFIG_H

#include "commonBased.h"


    class Config
    {

    private:
        static std::shared_ptr<Config> config_;//智能指针
        cv::FileStorage file_;

        Config () {} // private constructor makes a singleton
    public:
        ~Config();  // close the file when deconstructing

        // set a new config file
        static void setParameterFile( const std::string& filename );

        // access the parameter values
        template< typename T >
        static T get( const std::string& key )
        {
            return T( Config::config_->file_[key] );
        }

    };



#endif //RGB_D_VO_ORIGIN_CONFIG_H
