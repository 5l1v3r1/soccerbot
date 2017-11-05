/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   audio_packet_queue.h
 * Author: arnav
 *
 * Created on July 29, 2017, 2:14 PM
 */

#ifndef AUDIO_PACKET_QUEUE_H
#define AUDIO_PACKET_QUEUE_H

// Doesn't require security. 
// Helper function will be used for the priority queue. 
/*bool operator< (const AudioPacket& x, const AudioPacket& y) {   
    return x.get_priority() < y.get_priority();
}*/

class AudioPacketQueue{ 
    //private: 
    AudioPacketQueue();
    ~AudioPacketQueue();
    //std::priority_queue<template class AudioPacket > pq;
        
    public:     
    void update_priority();  
    void push(); 
    void pop ();
    void clear(); 
};


#endif /* AUDIO_PACKET_QUEUE_H */
