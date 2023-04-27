#include <bitset>
#include <map>
#include <cmath>
#include "ooo_cpu.h"

using namespace std;

#define no_of_bimodal_entries   16000

#define no_of_tagged_components   12

int TAGGED_TABLE_SIZE[] ={9,9,10,10,10,10,11,11,11,11,10,10};
int TAG_SIZE[] = {15,14,13,12,12,11,10,9,8,8,7,7};
int GEOMETRIC_HISTORY[] = {640,430,254,160,101,64,40,25,16,10,6,4};

bitset<640> ghr;
int phr;

class TAGGED_PREDICTOR_ENTRIES{

    public:
    int pred,u;
    int tag;  
};

class CSR{

    public:
    int intial_length,target_length,value;
 
    void update(bitset<640> ghr){
        value = (value<<1) + ghr[0];

        value ^= (value  & (1<<target_length)) >> target_length; 

        value ^= (ghr[intial_length] << (intial_length%target_length));

        value &= (1<<target_length) -1 ;
    }     
};

class TAGE_PREDICTOR{

    public:

      // bimodal base predictor
      int bimodal_table[no_of_bimodal_entries];

      // tagged predictors
      TAGGED_PREDICTOR_ENTRIES* tagged_pred[no_of_tagged_components] ;
      int tagged_table_size[no_of_tagged_components],tag_size[no_of_tagged_components],geomtric_history[no_of_tagged_components];
      int tagged_index[no_of_tagged_components],tagged_tag[no_of_tagged_components];

      // circular shift registers
      CSR csrindex[no_of_tagged_components],csrtag[no_of_tagged_components][2];

      //
      bool primary_pred,alt_pred,final_pred;
      int primary_table,alt_table;
      int alt_better;
      int clock,clock_state;
      bool found1,found2;
      int pred_from;

};

void func(int *x,int max,bool taken){
    if(taken && *x<max){
        *x=*x+1;
    }
    else if(!taken && *x>0){
      *x =  *x-1;
    }
    return;
}

TAGE_PREDICTOR tage_predictor;

 void O3_CPU::initialize_branch_predictor()
 {
    TAGE_PREDICTOR *t = &tage_predictor;

    for(int i=0;i<no_of_tagged_components;i++){

        int tablesize = TAGGED_TABLE_SIZE[i];

        t->tagged_table_size[i] = tablesize;
        t->tagged_pred[i] = new TAGGED_PREDICTOR_ENTRIES[1<<tablesize];
        
        for(int j=0;j<tablesize;j++){
            t->tagged_pred[i][j].pred = t->tagged_pred[i][j].tag = t->tagged_pred[i][j].u = 0;
        }

        t->tag_size[i] = TAG_SIZE[i];
        t->geomtric_history[i] = GEOMETRIC_HISTORY[i];

        t->tagged_index[i] = t->tagged_tag[i] = 0;

        t->csrindex[i].value = t->csrtag[i][0].value = t->csrtag[i][1].value = 0;

        t->csrindex[i].intial_length = t->csrtag[i][0].intial_length = t->csrtag[i][1].intial_length = t->geomtric_history[i];

        t->csrindex[i].target_length = t->csrtag[i][0].target_length = t->tag_size[i]; t->csrtag[i][1].target_length = t->tag_size[i]-1;
    }

    for(int i=0;i<no_of_bimodal_entries;i++){
        t->bimodal_table[i] = 2;
    }
  
    t->primary_pred = t->alt_pred = t->final_pred = false;
    t->primary_table = t->alt_table = no_of_tagged_components;

    t->clock = t->clock_state = 0;

    t->alt_better = 8;
    phr = 0;
    ghr.reset();
      
 }

uint8_t O3_CPU::predict_branch(uint64_t ip, uint64_t predicted_target, uint8_t always_taken, uint8_t branch_type)
{
TAGE_PREDICTOR *t = &tage_predictor;

    for(int i=0;i<no_of_tagged_components;i++){
        

        t->tagged_index[i] = ip ^ (ip>>t->tagged_table_size[i]) ^ (t->csrindex[i].value) ^ phr ^ (phr & 0);
        t->tagged_index[i] = t->tagged_index[i] & ((1<< t->tagged_table_size[i])-1);

        t->tagged_tag[i] = ip ^ (t->csrtag[i][0].value) ^ (t->csrtag[i][1].value << 1);
        t->tagged_tag[i] = t->tagged_tag[i] & ((1<< t->tagged_table_size[i]) - 1);
    }

    t->found1 = false; t->found2 = false;bool alt_is_better = false;

    for(int i=0;i<no_of_tagged_components;i++){
        if(!t->found1){
              if(t->tagged_pred[i][t->tagged_index[i]].tag == t->tagged_tag[i]){
                t->primary_table = i;
                t->primary_pred = (t->tagged_pred[i][t->tagged_index[i]].pred >= 4);
                if((t->tagged_pred[i][t->tagged_index[i]].pred==4 || t->tagged_pred[i][t->tagged_index[i]].pred == 3) && t->tagged_pred[i][t->tagged_index[i]].u == 0){
                    alt_is_better = true;
                }
                t->found1 = true;
              }
        }
        else if(!t->found2){
              if(t->tagged_pred[i][t->tagged_index[i]].tag == t->tagged_tag[i]){
                t->alt_table = i;
                t->alt_pred = (t->tagged_pred[i][t->tagged_index[i]].pred >= 4);
                t->found2 = true;
               
              }
        }
       else if(t->found2==true){
        break;
       }
      
    }

    

     bool bimodal_pred = (t->bimodal_table[ip%no_of_bimodal_entries] > 1);

    if(t->found2){
        if(alt_is_better){t->final_pred = t->alt_pred;t->pred_from = 1;}
        else{t->final_pred = t->primary_pred;t->pred_from = 0;}
    }
    else if(t->found1){
        if(alt_is_better){t->final_pred = t->alt_pred=bimodal_pred;t->pred_from = 1;}
        else{t->final_pred = t->primary_pred;t->pred_from = 0;}
    }
    else{
        t->final_pred = bimodal_pred;t->pred_from = 0;
    }

    return t->final_pred;

}

void O3_CPU::last_branch_result(uint64_t ip, uint64_t branch_target, uint8_t taken, uint8_t branch_type)
{
   TAGE_PREDICTOR *t = &tage_predictor;

   if(t->found1){
        func(&t->tagged_pred[t->primary_table][t->tagged_index[t->primary_table]].pred,7,taken);

        if(t->final_pred!=t->alt_pred){
            func(&t->tagged_pred[t->primary_table][t->tagged_index[t->primary_table]].u,3,t->primary_pred == taken);
        }
   }
   
   if(t->found2 && t->tagged_pred[t->primary_table][t->tagged_index[t->primary_table]].u == 0){
        func(&t->tagged_pred[t->alt_table][t->tagged_index[t->alt_table]].pred,7,taken);
   }

    func(&t->bimodal_table[ip%no_of_bimodal_entries],3,taken);
   

   if(t->pred_from == 1 && t->primary_pred != t->alt_pred){
        func(&t->alt_better,15,t->alt_pred==taken);
   }

   if(t->final_pred!=taken && (t->primary_table != 0||!t->found1)){
        
        int k=-1;
        if(!t->found1){t->primary_table = no_of_tagged_components;}

        for(int i=0;i<t->primary_table;i++){
            if(t->tagged_pred[i][t->tagged_index[i]].u == 0){
                k = i;break;
            }
            else{
               t->tagged_pred[i][t->tagged_index[i]].u--; 
            }
        }
       
       if(k!=-1){
        for(int i=0;i<t->primary_table;i++){
            if(i<k){
                 t->tagged_pred[i][t->tagged_index[i]].u++; 
            }
            else if(t->tagged_pred[i][t->tagged_index[i]].u==0 && !(rand()%no_of_tagged_components)){
                
                t->tagged_pred[i][t->tagged_index[i]].pred = 3+taken;
                t->tagged_pred[i][t->tagged_index[i]].tag = t->tagged_tag[i]; 

                break;
            }
        }
       } 
   }

   ghr = ghr << 1; ghr.set(0,taken);

   phr = phr & (111111111111111);
   phr = phr << 1;phr = phr + taken;
   
   for(int i=0;i<no_of_tagged_components;i++){

     t->csrindex[i].update(ghr);
     t->csrtag[i][0].update(ghr);
     t->csrtag[i][1].update(ghr);
   }

    t->clock++;

   if(t->clock==20){
    t->clock = 0;
    t->clock_state = 1-t->clock_state;
    for(int i=0;i<no_of_tagged_components;i++){
        for(int j=0;j<t->tagged_table_size[i];j++){
            t->tagged_pred[i][j].u &= (t->clock_state+1);
        }
    }
   }

}

