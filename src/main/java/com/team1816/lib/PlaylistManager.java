package com.team1816.lib;

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

    /**
     * Instantiates the Playlist manager
     */
    @Inject
    public PlaylistManager() {
        songChooser = new SendableChooser<>();
        SmartDashboard.putData("Song", songChooser);
        for (Playlist playlist : Playlist.values()) {
            songChooser.addOption(playlist.name(), playlist);
        }
        songChooser.setDefaultOption(Playlist.COCONUT_MALL.name(), Playlist.COCONUT_MALL);

    }

    /**
     * Updates the song to the desiredSong
     *
     * @return songChanged
     */
    public boolean update() {
        Playlist selectedSong = songChooser.getSelected();
        boolean songChanged = desiredSong != selectedSong;

        if (songChanged) {
            GreenLogger.log("Song changed from: " + desiredSong + ", to: " + selectedSong.name());
            desiredSong = selectedSong;
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
        //TODO add  putData() with sendable later
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
        return "/songs/" + path;
    }

    /**
     * Enum for songs
     */
    private enum Playlist {
        COCONUT_MALL,
        TIMBER_PITBULL,
    }



}
