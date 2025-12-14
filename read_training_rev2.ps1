
Add-Type -AssemblyName System.Speech
$synth = New-Object System.Speech.Synthesis.SpeechSynthesizer

# Tentative de sélection d'une voix française
$frVoice = $synth.GetInstalledVoices() | Where-Object { $_.VoiceInfo.Culture -like "fr-*" } | Select-Object -First 1

if ($frVoice) { 
    $synth.SelectVoice($frVoice.VoiceInfo.Name) 
    Write-Host "Voix française sélectionnée : $($frVoice.VoiceInfo.Name)"
}
else {
    Write-Host "Aucune voix française trouvée, utilisation de la voix par défaut (anglaise probablement)."
}

$path = "C:\Users\basti\.gemini\antigravity\brain\e907003b-84bd-4188-b3ce-c5c555d31ee0\training_details_rev2.md"

if (Test-Path $path) {
    $text = Get-Content $path -Raw
    
    # Nettoyage basique du Markdown pour fluidifier la lecture audio
    $text = $text -replace "#+", ""         # Enlever les # des titres
    $text = $text -replace "\*\*", ""       # Enlever les ** du gras
    $text = $text -replace "`", ""          # Enlever les backticks de code
    $text = $text -replace "\[.*?\]", ""    # Enlever les liens [Texte]
    $text = $text -replace "\(.*?\)", ""    # Enlever les liens (URL)
    
    Write-Host "Lecture en cours..."
    $synth.Speak("Vision globale de l'entraînement Révision 2.")
    $synth.Speak($text)
    Write-Host "Lecture terminée."
} else {
    Write-Error "Fichier introuvable : $path"
}
