# GNZDA_to_UTC
Insertion du message $UTC avec correction en temps réel du décalage avec le signal PPS.

Permet de bien faire fonctionner certains équipements qui demande le message $UTC d'Applanix, mais en utilisant un GPS générique.

Le point décimal de l'heure UTC compensée dans le message $UTC est synchro à ± 0,5 mS avec l'heure réelle UTC.
Vous pouvez donc vous servir de ce repère comme d'un émulateur du signal PPS bon marché. Ce n'est pas un signal au TOP de la seconde (! TOW).

À titre indicatif seulement :
Un test avec un analyseur logique et le GPS U-Blox ZED-F9R, montre que le premier caractère après le signal PPS, arrive environ 237 mS plus tard.
Ce délai est variable et dépend de la charge de traitement en temps réel du GPS et de son paramétrage.
Il n'est donc pas indiqué de se servir du premier caractère reçu comme d'une émulation du signal PPS.