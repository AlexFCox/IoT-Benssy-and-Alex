# IoT-Gruppenarbeit

## Miro Board für Flowchart Dokumentation
https://miro.com/app/board/uXjVJ4x2e0A=/?focusWidget=3458764644625770545

- Setup Loop enthält alle Variabeln
- Main Loop besteht aus 7 Schritten

(Effort Values nach T-Shirt sizes)

### 1. Sensor einlesen
- Effort: S
- Description: Wird bereits in erstem Schritt mit dem Code grösstenteils abgedeckt. Set up intiial transfer of data.
  
### 2. Sensor Daten an moving Average buffer hinzufügen
- Effort: S
- Description: Ist für den zweiten Schritt nötig.
  
### 3. Moving Average berechnen
- Effort: S
- Description: Ist ebenfalls für den zweiten Schritt nötig. Der Highpass filter diesbezüglich wird erst in Schritt 4 for die Hysterese benutzt.
  
### 4. Detect Rep Phase (inkl. Hysterese)
- Effort: L
- Description: Ist für den dritten und vierten Schritt nötig. Hier filtern wir Reps damit wir nur solche bekommen die auch teil eines Trainings sind.

### 5. Count Valid Rep 
- Effort: M
- Description: Ist für den dritten und vierten Schritt nötig. Hier hören wir auf die Hysterese dass ein Rep richtig gezählt wird. Auch die parameter für die Hysterese sind unterschiedlich ob man gerade "lupft" oder "fallen lässt" die Hantel.
  
### 6. Check training status 
- Effort: M
- Description: Ist für den dritten Schritt nötig um sicherzustellen wann wir in einem Training sind und wann nicht.
  
### 7. Update Cloud Variables & handle Kadenz 
- Effort: M
- Description: Für den fünften Schritt wichtig. 

## Known Improvements TBD
- When picking up dumbbell, single rep already counted -> Check if hysterese here is good. Maybe not implemented correctly or need to tweak value.