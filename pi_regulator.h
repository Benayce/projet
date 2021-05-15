/*Pour cette partie nous avons reutilise le code du TP4
 * lequel nous avons modifie pour notre projet
 *
 */

#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread
void pi_regulator_start(void);

//gives the state of the pi regulator
uint8_t get_state(void);

#endif /* PI_REGULATOR_H */
