# Guide "Pour Moi": Compiler OpenCV avec CUDA (70-80 FPS)

**Objectif**: Activer la puissance de votre RTX 4070 pour passer de 10 FPS à 80 FPS.
**Temps estimé**: 2-3 heures (dont 90% d'attente pendant la compilation).

---

## ✅ Ce que nous avons déjà préparé

1. **Code Source OpenCV**: Vous l'avez déjà dans `C:\opencv\sources`
2. **Modules Contrib**: Je viens de les télécharger pour vous dans `C:\Users\basti\source\repos\RealsenseBodyPose\opencv_build\opencv_contrib-4.10.0`
3. **Outils**: Visual Studio et CMake sont déjà installés.

---

## Étape 1: Préparer le dossier de construction

Ouvrez **PowerShell en tant qu'administrateur** et lancez ces commandes pour créer un dossier propre:

```powershell
cd C:\
mkdir opencv_cuda_build
cd opencv_cuda_build
```

---

## Étape 2: Configurer avec CMake (La partie cruciale)

1. Ouvrez l'application **CMake (cmake-gui)**.
   *(Cherchez "CMake" dans le menu Démarrer ou lancez `cmake-gui`)*

2. **Remplissez les chemins en haut**:
   - **Where is the source code**: `C:/opencv/sources`
   - **Where to build the binaries**: `C:/opencv_cuda_build`

3. Cliquez sur le bouton **"Configure"** (en bas).
   - Une fenêtre s'ouvre.
   - Sélectionnez **"Visual Studio 17 2022"**.
   - Platform: **"x64"**.
   - Cliquez sur **"Finish"**.
   - *Attendez que la barre de chargement finisse.*

4. **Modifiez les options (Barre de recherche en haut)**:

   Recherchez et **COCHEZ** ces cases (essentiel pour la robotique/fiducial):
   - [x] `WITH_CUDA`
   - [x] `OPENCV_DNN_CUDA`
   - [x] `ENABLE_FAST_MATH`
   - [x] `BUILD_opencv_world`
   - [x] `BUILD_opencv_rgbd` (Pour RealSense/Kinect)
   - [x] `BUILD_opencv_tracking` (Suivi d'objets)
   - [x] `BUILD_opencv_aruco` (Marqueurs fiducials - parfois inclus dans objdetect, à vérifier)
   - [x] `BUILD_opencv_optflow` (Flux optique pour mouvement)

   Recherchez et **MODIFIEZ** ces valeurs (cliquez sur la valeur pour éditer):
   - `OPENCV_EXTRA_MODULES_PATH` ➔ `C:/Users/basti/source/repos/RealsenseBodyPose/opencv_build/opencv_contrib-4.10.0/modules`
   - `CUDA_ARCH_BIN` ➔ `8.9`  *(C'est l'architecture spécifique de votre RTX 4070)*
   - `CMAKE_INSTALL_PREFIX` ➔ `C:/opencv_cuda_install`

   *(Optionnel mais recommandé pour aller plus vite)*:
   - Décochez `BUILD_TESTS`
   - Décochez `BUILD_PERF_TESTS`
   - Décochez `BUILD_EXAMPLES`

5. Cliquez sur **"Configure"** à nouveau.
   - *Attendez...*
   - Vérifiez dans le texte blanc en bas que vous voyez:
     `CUDA: YES (ver 12.6 ...)`
     `cuDNN: YES (ver 8.9 ...)`

6. Cliquez sur **"Generate"**.

7. Cliquez sur **"Open Project"**. Cela va ouvrir Visual Studio.

---

## Étape 3: Compiler (Le moment "Pause Café")

1. Dans Visual Studio qui vient de s'ouvrir:
   - En haut, changez "Debug" en **"Release"**.

2. Dans le panneau de droite ("Explorateur de solutions"):
   - Faites un **Clic-Droit** sur **"ALL_BUILD"**.
   - Cliquez sur **"Générer"** (ou "Build").

3. **ATTENDEZ**.
   - Cela va prendre entre 1h et 2h.
   - Vous verrez défiler des lignes dans la fenêtre "Sortie".
   - Tant qu'il n'y a pas d'erreur rouge "FAILED", tout va bien.

---

## Étape 4: Installer

Une fois la compilation finie (vous verrez "Génération: X réussis, 0 échoués"):

1. Toujours dans Visual Studio, panneau de droite.
2. Cherchez le projet **"INSTALL"** (souvent vers le bas).
3. **Clic-Droit** sur **"INSTALL"** ➔ **"Générer"**.

Cela va copier tous les fichiers finaux dans `C:\opencv_cuda_install`.

---

## Étape 5: Activer le Turbo dans votre Application

Maintenant que vous avez un OpenCV dopé au CUDA, il faut dire à votre projet de l'utiliser.

1. **Supprimez votre ancien dossier de build**:
   ```powershell
   cd C:\Users\basti\source\repos\RealsenseBodyPose
   Remove-Item build -Recurse -Force
   mkdir build
   cd build
   ```

2. **Configurez votre projet avec le nouvel OpenCV**:
   ```powershell
   cmake .. -G "Visual Studio 17 2022" -A x64 -DOpenCV_DIR="C:/opencv_cuda_install"
   ```
   *(Notez l'argument `-DOpenCV_DIR` qui pointe vers votre nouvelle installation)*

3. **Compilez votre application**:
   ```powershell
   cmake --build . --config Release -j 8
   ```

---

## Étape 6: Le Grand Test

Lancez votre application:

```powershell
cd bin\Release
.\RealsenseBodyPose.exe --model ..\..\..\models\yolov8n-pose.onnx
```

Regardez la console. Vous devriez voir:
✅ **[INFO] GPU: NVIDIA GeForce RTX 4070**
✅ **FPS: 70+**

---

## En cas de problème

Si vous avez une erreur "DLL not found":
- Copiez `C:\opencv_cuda_install\x64\vc17\bin\opencv_world4100.dll` à côté de votre `.exe`.

Si CMake ne trouve pas CUDA:
- Vérifiez que vous avez bien installé le "NVIDIA CUDA Toolkit 12.6".

**Bonne chance ! C'est l'étape ultime pour la performance.**
