Questo é un mio progetto realizzato con il software PROCESSING durante i miei studi universitari.

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Per chi non conoscesse il software PROCESSING:
E'un ambiente di sviluppo open source utilizzato per creare applicazioni grafiche interattive, visualizzazioni di dati, animazioni, arte generativa e altro ancora. Si basa sul linguaggio di programmazione Java ed è progettato per essere facile da imparare e usare anche per chi non ha una conoscenza approfondita della programmazione.Processing fornisce una vasta gamma di funzioni e librerie grafiche per rendere la creazione di immagini e animazioni più facile e veloce. Inoltre, è possibile integrare librerie di terze parti per estendere le funzionalità di base di Processing.E'stato creato per essere utilizzato sia da artisti che da programmatori e per supportare una vasta gamma di applicazioni, dall'arte digitale all'elaborazione di dati scientifici. È disponibile gratuitamente e può essere eseguito su diverse piattaforme, tra cui Windows, Mac OS X e Linux.

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Questo sketch effettua un controllo sulla base della stima della posa dell'uniciclo ottenuta mediante un filtro di Kalman esteso che fonde l'odometria (movimento ruote) con le misure da un certo numero di landmark,il filtro di Kalman usa solo la predizione basata sui passi encoder delle ruote.
La misura associata a un landmark visibile è l'angolo (detto di bearing) con cui il robot vede questo landmark rispetto al suo orientamento.
Ergo, si disegna un robot mobile di tipo 'uniciclo' e lo si controlla in velocità (senza limiti di saturazione) verso il punto cliccato col mouse mediante la legge di controllo proporzionale, tuttavia il calcolo del controllo è fatto sulle variabili stimate anzichè su quelle vere.
Sulla schermata viene anche disegnato (in giallo) il robot nella posizione stimata (quello vero è disegnato in rosso).
In più, dei triangoli colorati col bordo rosso rappresentano la posizione dei landmark presenti nell'ambiente. 
All'interno di ciascun triangolo viene anche riportato un identificatore del tipo 'Li' per il landmark i-esimo.
I landmark sono visibili e quindi bianchi solo quando cadono dentro un settore circolare di raggio rMax, angolo betaMax e sono accesi(angolo centrato nell'uniciclo e simmetrico rispetto alla direzione di avanzamento dell'uniciclo stesso).
Perciò quando un landmark diventa visibile (cioè cade nella regione di visibilità ed è acceso), si può utilizzare l'angolo di bearing da tale landmark.
Tale angolo va considerato positivo quando il landmark è sulla sinistra del robot e negativo quando è sulla destra (vale 0 se il landmark sta proprio nella direzione verso cui è orientato l'uniciclo). 
Tale misura è caratterizzata da un errore gaussiano a media nulla e deviazione standard che va presa pari a 10 gradi.
Ricapitolando i landmark che cadono nella regione di visibilità sono evidenziati poichè viene modificato il colore in bianco,i landmark spenti da tastiera premendo il numero corrispondente sono evidenziati poichè viene modificato il colore in nero mentre quelli accesi ma non visibili sono di colore grigio.
Premendo i tasti 'r' e 'R' è possibile diminuire e, rispettivamente, aumentare il range massimo rMax del settore circolare.
Analogamente, con i tasti 'b' e 'B', è possibile variare l'angolo di bearing massimo betaMax che definisce l'apertura del settore circolare
(nel senso che l'angolo di apertura di tale settore è 2*betaMax).
Il valore minimo positivo per rangeMax è 50 pixel e i valori dell'angolo betaMax sono compresi tra :  10 < betaMax < 280 gradi. 
Con le frecce è possibile modificare la frequenza delle misure di distanza dai 
landmark visibili o, più precisamente, il tempo 'tStep' che intercorre tra una misura e la 
successiva (comunque non inferiore al ciclo di Processing, per default dt = 1/60 s):
con le frecce SU e GIU' si incrementa e decrementa rispettivamente di 1 tale tempo 
mentre con le frecce DESTRA e SINISTRA lo si moltiplica e divide rispettivamente per 2
(per modificarlo più velocemente).
Quando tale tempo è molto grande la ricostruzione diviene puramente odometrica.
Sulla schermata vengono riportati: le coordinate (x, y, theta) dell'uniciclo,
le coordinate (xDes, Ydes) del punto da raggiungere e il tempo impiegato per farlo.
Inoltre si riportano le velocità angolari (omegaR, OmegaL) delle due ruote,la velocità longitudinale v1 e quella angolare v2.
Infine si indicano anche le grandezze stimate (xHat, yHat, thetaHat) con la loro matrice di covarianza 'P' e il tempo 'tStep' (modificabile da tastiera) che intercorre tra una lettura e la successiva.
