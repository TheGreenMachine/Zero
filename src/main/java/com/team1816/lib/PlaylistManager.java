package com.team1816.lib;

import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.inject.Inject;
import javax.inject.Singleton;

/**
 * A manager for song selection using CTRE's CHIRP files
 * @see com.ctre.phoenix.music.Orchestra
 */
@Singleton
public class PlaylistManager {
    /**
     * Properties: Selection
     */
    protected SendableChooser<Playlist> songChooser;
    protected Playlist desiredSong;

    private Drive drive;

    /**
     * Instantiates the Playlist manager
     */
    @Inject
    public PlaylistManager() {
        drive = (Injector.get(Drive.Factory.class)).getInstance();
        songChooser = new SendableChooser<>();
        SmartDashboard.putData("Song", songChooser);
        for (Playlist playlist : Playlist.values()) {
            songChooser.addOption(playlist.name(), playlist);
        }
        songChooser.setDefaultOption(Playlist.COCONUT_MALL.name(), Playlist.COCONUT_MALL);
    }

    /**
     * Updates the song to the desiredSong and stops the song if it has been playing for too long
     *
     * @return songChanged
     */
    public boolean update() {
        Playlist selectedSong = songChooser.getSelected();
        boolean songChanged = desiredSong != selectedSong;

        if (songChanged) {
            GreenLogger.log("Song changed from: " + desiredSong + " to: " + selectedSong.name());
            desiredSong = selectedSong;

            drive.gaudette.loadMusic(getFilePath());
        }

        if (drive.gaudette.getCurrentTime() > 10) {
            drive.gaudette.stop();
        }

        return songChanged;
    }

    /**
     * Outputs values to SmartDashboard
     */
    public void outputToSmartDashboard() {
        if (desiredSong != null) {
            SmartDashboard.putString("SongSelected", desiredSong.name());
        }
    }


    /**
     * Returns the file path to the desired song
     *
     * @return songPath
     */
    public String getFilePath() {
        String path = "";
        switch (desiredSong) {
            case COCONUT_MALL -> path = "coconutMall";
            case TIMBER_PITBULL -> path = "timber";
        }
        return "src/main/resources/songs/" + path + ".chrp";
    }

    /**
     * Enum for songs
     */
    private enum Playlist {
        COCONUT_MALL,
        TIMBER_PITBULL,
    }
}
