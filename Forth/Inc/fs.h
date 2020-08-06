/*
 * fs.h
 *
 *  Created on: 17.02.2020
 *      Author: psi
 */

#ifndef INC_FS_H_
#define INC_FS_H_


void FS_init(void);
void FS_include  (uint8_t *str, int count);
void FS_cat      (void);
void FS_ls       (void);
void FS_cd       (void);
void FS_pwd      (void);

void FS_evaluate (uint8_t* str, int count);
void FS_type     (uint8_t* str, int count);
void FS_cr       (void);
int  FS_token    (uint8_t **str);

#endif /* INC_FS_H_ */
